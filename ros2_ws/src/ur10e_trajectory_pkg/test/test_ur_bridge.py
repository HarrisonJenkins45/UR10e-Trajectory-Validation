"""ur_bridge against a fake UR controller that speaks the real wire protocol.

The fake serves 1116-byte realtime packets on every connection, runs the
streaming program's side of the handshake (connect back, read velocity
vectors) when it receives that program, serves RTDE input registers,
executes one-line speedj programs, answers the dashboard server, and
integrates the resulting velocity into its joint positions. It can be told
to misbehave: registers held by another session, an arm held back, a program
that ignores zeros and stopj, motion from elsewhere, a protective stop. It
checks the PC side end to end; it cannot check URScript semantics on a real
controller.
"""

import contextlib
import re
import socket
import struct
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float64MultiArray  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from ur10e_trajectory_pkg import hardware_protocol as wire  # noqa: E402
from ur10e_trajectory_pkg import ur_bridge  # noqa: E402

PACKET_DOUBLES = 139


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def listening_socket(backlog):
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 0))
    server.listen(backlog)
    return server, server.getsockname()[1]


class FakeUR:
    def __init__(self, scaling=1.0, highest_register=47, in_use_attempts=0, follow=1.0,
                 stuck=False):
        self.scaling = scaling
        # Older controllers have RTDE input registers 0..23 only.
        self.highest_register = highest_register
        # Input setups refused as IN_USE (a previous session) before success.
        self.in_use_attempts = in_use_attempts
        # Fraction of the target velocity the joints deliver (1 = all).
        self.follow = follow
        # A program that ignores zero velocity and stopj; only a dashboard
        # stop ends it.
        self.stuck = stuck
        self.robot_mode, self.safety_mode = 7, 1          # RUNNING, NORMAL
        self.q = np.zeros(6)
        self.target = np.zeros(6)
        self.vectors = 0
        self.programs = []
        self.dashboard_commands = []
        self.running = True
        self.server, self.port = listening_socket(8)
        self.rtde_server, self.rtde_port = listening_socket(2)
        self.dashboard_server, self.dashboard_port = listening_socket(4)
        self.rtde_program = False
        self.program_state = 1                  # STOPPED until a program arrives
        for target in (self.accept, self.serve_rtde, self.integrate, self.serve_dashboard):
            threading.Thread(target=target, daemon=True).start()

    def packet(self):
        values = np.zeros(PACKET_DOUBLES)
        values[0] = time.monotonic()
        values[7:13] = self.target
        values[31:37] = self.q
        values[37:43] = self.target * self.follow
        values[94], values[101] = self.robot_mode, self.safety_mode
        values[117], values[131] = self.scaling, self.program_state
        return struct.pack('>I', 4 + 8 * PACKET_DOUBLES) + struct.pack(
            f'>{PACKET_DOUBLES}d', *values)

    def integrate(self):
        while self.running:
            self.q = self.q + self.target * self.follow * 0.008
            time.sleep(0.008)

    def accept(self):
        while self.running:
            try:
                connection, _ = self.server.accept()
            except OSError:
                return
            threading.Thread(target=self.serve, args=(connection,), daemon=True).start()

    def serve(self, connection):
        def send_state():
            try:
                while self.running:
                    connection.sendall(self.packet())
                    time.sleep(0.008)
            except OSError:
                pass

        threading.Thread(target=send_state, daemon=True).start()
        buffer = b''
        try:
            while self.running:
                data = connection.recv(65536)
                if not data:
                    return
                buffer += data
                if b'def ros_speedj_rtde' in buffer and b'\nend\n' in buffer:
                    self.programs.append('rtde')
                    self.rtde_base = int(re.search(
                        rb'read_input_integer_register\((\d+)\)', buffer).group(1))
                    self.rtde_program = True
                    self.program_state = 2
                    buffer = b''
                if b'def ros_speedj_stream' in buffer and b'\nend\n' in buffer:
                    self.programs.append('stream')
                    self.program_state = 2
                    host, port = re.search(rb'socket_open\("([\d.]+)", (\d+)', buffer).groups()
                    threading.Thread(target=self.stream, args=(host.decode(), int(port)),
                                     daemon=True).start()
                    buffer = b''
                for match in re.finditer(rb'speedj\(\[([^\]]*)\]', buffer):
                    self.programs.append('line')
                    self.program_state = 2
                    self.set_target([float(v) for v in match.group(1).split(b',')])
                if b'stopj' in buffer:
                    self.programs.append('stopj')
                    if not self.stuck:
                        self.target = np.zeros(6)
                        self.rtde_program = False
                # Keep a program that has not fully arrived; drop the rest.
                if not (buffer.startswith(b'def ') and b'\nend\n' not in buffer):
                    buffer = b''
        except OSError:
            pass
        finally:
            connection.close()

    def set_target(self, values):
        values = np.asarray(values, dtype=float)
        if self.stuck and not np.any(values):
            return
        self.target = self.scaling * values

    def stream(self, host, port):
        sock = socket.create_connection((host, port), timeout=2.0)
        pending = b''
        while self.running:
            try:
                data = sock.recv(4096)
            except OSError:
                return
            if not data:
                return
            pending += data
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                self.set_target([float(v) for v in line.strip(b'()').split(b',')])
                self.vectors += 1

    def serve_rtde(self):
        """Minimal RTDE: accept the handshake, apply register writes with a new sequence."""
        try:
            connection, _ = self.rtde_server.accept()
        except OSError:
            return
        last_sequence = None
        try:
            while self.running:
                kind, payload = wire.rtde_read(connection)
                if kind == wire.RTDE_REQUEST_PROTOCOL_VERSION:
                    connection.sendall(wire.rtde_packet(kind, b'\x01'))
                elif kind == wire.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
                    connection.sendall(wire.rtde_packet(kind, b'\x01DOUBLE'))
                elif kind == wire.RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
                    numbers = [int(name.rsplit(b'_', 1)[1]) for name in payload.split(b',')]
                    refusal = None
                    if max(numbers) > self.highest_register:
                        refusal = b'NOT_FOUND'
                    elif self.in_use_attempts > 0:
                        self.in_use_attempts -= 1
                        refusal = b'IN_USE'
                    if refusal:
                        connection.sendall(wire.rtde_packet(
                            kind, b'\x00' + b','.join([refusal] * len(numbers))))
                        connection.close()
                        return self.serve_rtde()
                    connection.sendall(wire.rtde_packet(
                        kind, b'\x02' + b','.join([b'DOUBLE'] * 6 + [b'INT32'])))
                elif kind == wire.RTDE_CONTROL_PACKAGE_START:
                    connection.sendall(wire.rtde_packet(kind, b'\x01'))
                elif kind == wire.RTDE_DATA_PACKAGE:
                    values = struct.unpack('>6di', payload[1:])
                    if values[6] != last_sequence and self.rtde_program:
                        last_sequence = values[6]
                        self.set_target(values[:6])
                        self.vectors += 1
        except (OSError, ConnectionError):
            pass

    def serve_dashboard(self):
        while self.running:
            try:
                connection, _ = self.dashboard_server.accept()
            except OSError:
                return
            with connection:
                try:
                    connection.sendall(b'Connected: Universal Robots Dashboard Server\n')
                    command = connection.makefile('rb').readline().decode().strip()
                    self.dashboard_commands.append(command)
                    if command == 'stop':
                        self.target = np.zeros(6)
                        self.rtde_program = False
                        self.program_state = 1
                        connection.sendall(b'Stopped\n')
                    elif command == 'is in remote control':
                        connection.sendall(b'true\n')
                    else:
                        connection.sendall(b'could not understand\n')
                except OSError:
                    pass

    def close(self):
        self.running = False
        for server in (self.server, self.rtde_server, self.dashboard_server):
            server.close()


def wait_until(condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return condition()


def stream(publisher, velocity, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        publisher.publish(Float64MultiArray(data=list(velocity)))
        time.sleep(0.05)


WRIST = [0, 0, 0, 0, 0, 0.05]
ZERO = [0.0] * 6


@contextlib.contextmanager
def bridge_for(fake, stream_kind='rtde', **extra):
    params = {'robot_ip': '127.0.0.1', 'port': fake.port, 'enable_commands': True,
              'stream': stream_kind, 'host_ip': '127.0.0.1', 'stream_port': free_port(),
              'rtde_port': fake.rtde_port, 'dashboard_port': fake.dashboard_port, **extra}
    args = ['--ros-args']
    for name, value in params.items():
        args += ['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}']
    rclpy.init(args=args)
    bridge = ur_bridge.URBridge()
    # The executor's node name: the only publisher the bridge accepts.
    driver = rclpy.create_node('plan_executor')
    publisher = driver.create_publisher(Float64MultiArray, '/ur/joint_velocity_command', 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(driver)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        yield bridge, publisher
    finally:
        executor.shutdown()
        bridge.shutdown()
        bridge.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
        fake.close()


def ready(bridge):
    return wait_until(lambda: bridge.status() == 'ready', 6.0)


@pytest.mark.parametrize('stream_kind, highest_register', [
    ('rtde', 47), ('rtde', 23), ('program', 47), ('lines', 47)])
def test_bridge_delivers_velocity_and_a_stalled_stream_faults_and_stops(
        stream_kind, highest_register):
    fake = FakeUR(highest_register=highest_register)
    with bridge_for(fake, stream_kind) as (bridge, publisher):
        assert ready(bridge), bridge.status()
        start = fake.q.copy()
        stream(publisher, WRIST, 2.0)
        moved = fake.q - start
        assert bridge.fault is None
        # The stream stops without a zero: a latched fault, and a stopped arm.
        assert wait_until(lambda: bridge.fault is not None, 1.0)
        assert 'no arm command' in bridge.fault
        assert wait_until(lambda: bridge.state == 'idle', 2.0)
        np.testing.assert_allclose(fake.target, 0.0)
    assert moved[5] == pytest.approx(0.10, abs=0.03)
    np.testing.assert_allclose(moved[:5], 0.0, atol=1e-6)
    if stream_kind == 'program':
        assert fake.programs.count('stream') == 1 and fake.vectors >= 30
    if stream_kind == 'rtde':
        assert fake.programs.count('rtde') == 1 and fake.vectors >= 30
        # An older controller without registers 24..29 gets the lower set.
        assert fake.rtde_base == (24 if highest_register >= 29 else 18)


def test_a_zero_command_ends_in_a_verified_stop_without_a_fault():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 0.5)
        assert bridge.state == 'moving'
        stream(publisher, ZERO, 0.4)
        assert wait_until(lambda: bridge.state == 'idle', 1.0)
        assert bridge.fault is None and bridge.status() == 'ready'
        np.testing.assert_allclose(fake.target, 0.0)


def test_reset_after_a_stall_reverifies_the_arm_and_accepts_commands_again():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 0.5)
        assert wait_until(lambda: bridge.fault is not None, 1.0)
        assert wait_until(lambda: bridge.state == 'idle', 2.0)
        response = bridge.on_reset(Trigger.Request(), Trigger.Response())
        assert response.success and 'verifying the arm is still' in response.message
        assert ready(bridge), bridge.status()
        before = fake.q[5]
        stream(publisher, [0, 0, 0, 0, 0, -0.05], 0.6)
        assert fake.q[5] < before - 0.01 and bridge.fault is None


def test_registers_held_by_a_previous_session_are_retried_not_abandoned():
    fake = FakeUR(in_use_attempts=2)
    with bridge_for(fake) as (bridge, _):
        assert ready(bridge), bridge.status()
        assert fake.rtde_base == 24


def test_motion_nobody_commanded_is_stopped_and_latched():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, _):
        assert ready(bridge)
        fake.target = np.array([0, 0, 0, 0, 0, 0.1])   # another source moves the arm
        assert wait_until(lambda: bridge.fault is not None, 1.0)
        assert 'without a command' in bridge.fault
        assert wait_until(lambda: not np.any(fake.target), 1.0)
        assert bridge.status().startswith('fault')


def test_a_stop_the_program_ignores_escalates_to_a_dashboard_stop():
    fake = FakeUR(stuck=True)
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 0.6)
        publisher.publish(Float64MultiArray(data=ZERO))
        assert wait_until(lambda: 'stop' in fake.dashboard_commands, 4.0)
        assert 'stopj' in fake.programs
        assert 'dashboard stop' in bridge.fault
        assert wait_until(lambda: not np.any(fake.target), 1.0)


def test_an_arm_that_lags_its_command_faults():
    fake = FakeUR(follow=0.3)
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 2.8)
        assert bridge.fault is not None and 'delivers only 30%' in bridge.fault


def test_a_protective_stop_while_moving_faults():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 0.4)
        fake.safety_mode = 3
        stream(publisher, WRIST, 0.3)
        assert 'PROTECTIVE_STOP' in bridge.fault


def test_status_names_a_robot_that_is_not_running():
    fake = FakeUR()
    fake.robot_mode = 5
    with bridge_for(fake) as (bridge, publisher):
        assert wait_until(lambda: 'robot mode IDLE' in bridge.status(), 3.0), bridge.status()
        stream(publisher, WRIST, 0.3)
        assert fake.q[5] == 0.0


def test_another_publisher_on_the_command_topic_blocks_motion():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stranger = rclpy.create_node('forgotten_topic_pub')
        stranger.create_publisher(Float64MultiArray, '/ur/joint_velocity_command', 10)
        try:
            assert wait_until(lambda: bridge.check_command_sources() != [], 2.0)
            stream(publisher, WRIST, 0.5)
            assert fake.q[5] == 0.0
            assert 'forgotten_topic_pub' in bridge.status()
        finally:
            stranger.destroy_node()


def test_any_lifts_the_publisher_restriction_for_hand_run_checks():
    fake = FakeUR()
    with bridge_for(fake, allowed_command_nodes='any') as (bridge, _):
        assert ready(bridge)
        stranger = rclpy.create_node('forgotten_topic_pub')
        publisher = stranger.create_publisher(Float64MultiArray,
                                              '/ur/joint_velocity_command', 10)
        try:
            stream(publisher, WRIST, 0.6)
            assert fake.q[5] > 0.01 and bridge.fault is None
        finally:
            stranger.destroy_node()


def test_shutdown_stops_the_arm_and_ends_the_program():
    fake = FakeUR()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, WRIST, 0.5)
        bridge.shutdown()
        np.testing.assert_allclose(fake.target, 0.0)
        assert 'stopj' in fake.programs


def test_shadow_mode_sends_and_supervises_nothing():
    fake = FakeUR()
    with bridge_for(fake, enable_commands=False) as (bridge, publisher):
        time.sleep(0.5)
        fake.target = np.array([0, 0, 0, 0, 0, 0.1])   # the robot's own program
        stream(publisher, WRIST, 0.5)
        assert bridge.fault is None and fake.programs == []
        assert bridge.status().startswith('shadow mode')


def test_diagnosis_names_speed_scaling_and_a_stalled_controller():
    bridge = ur_bridge.URBridge.__new__(ur_bridge.URBridge)
    bridge.stream, bridge.stream_state = 'program', 'connected'
    state = {'speed_scaling': 0.3, 'program_state': 2.0}
    problems = bridge.diagnose(20, state, target=0.3, actual=0.3)
    assert any('speed scaling 0.30' in p for p in problems)
    assert any('targets only 30%' in p for p in problems)
    playing = {'speed_scaling': 1.0, 'program_state': 2.0}
    assert bridge.diagnose(20, playing, target=1.0, actual=0.4) == [
        'the controller targets the command, but the arm delivers only 40%: '
        'check safety mode and joint limits']
    assert any('only 5 commands' in p for p in bridge.diagnose(5, None, None, None))
    # Scaling reads 0 while no program plays; that is not a slowdown.
    assert not any('speed scaling' in p for p in bridge.diagnose(
        20, {'speed_scaling': 0.0, 'program_state': 1.0}, target=1.0, actual=1.0))
