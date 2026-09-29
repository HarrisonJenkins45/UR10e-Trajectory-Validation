"""ur_bridge against a fake UR controller that speaks the real wire protocol.

The fake serves 1116-byte realtime packets on every connection, runs the
streaming program's side of the handshake (connect back, read velocity
vectors) when it receives that program, serves RTDE input registers,
executes one-line speedj programs, and integrates the resulting velocity
into its joint positions. It checks the
PC side end to end; it cannot check URScript semantics on a real controller.
"""

import re
import socket
import struct
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float64MultiArray  # noqa: E402

from ur10e_trajectory_pkg import hardware_protocol as wire  # noqa: E402
from ur10e_trajectory_pkg import ur_bridge  # noqa: E402

PACKET_DOUBLES = 139


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class FakeUR:
    def __init__(self, scaling=1.0, highest_register=47):
        self.scaling = scaling
        # Older controllers have RTDE input registers 0..23 only.
        self.highest_register = highest_register
        self.q = np.zeros(6)
        self.target = np.zeros(6)
        self.vectors = 0
        self.programs = []
        self.running = True
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(8)
        self.port = self.server.getsockname()[1]
        self.rtde_server = socket.socket()
        self.rtde_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rtde_server.bind(('127.0.0.1', 0))
        self.rtde_server.listen(2)
        self.rtde_port = self.rtde_server.getsockname()[1]
        self.rtde_program = False
        self.program_state = 1                  # STOPPED until a program arrives
        threading.Thread(target=self.accept, daemon=True).start()
        threading.Thread(target=self.serve_rtde, daemon=True).start()
        threading.Thread(target=self.integrate, daemon=True).start()

    def packet(self):
        values = np.zeros(PACKET_DOUBLES)
        values[0] = time.monotonic()
        values[7:13] = self.target
        values[31:37] = self.q
        values[37:43] = self.target
        values[94], values[101] = 7, 1          # RUNNING, NORMAL
        values[117], values[131] = self.scaling, self.program_state
        return struct.pack('>I', 4 + 8 * PACKET_DOUBLES) + struct.pack(
            f'>{PACKET_DOUBLES}d', *values)

    def integrate(self):
        while self.running:
            self.q = self.q + self.target * 0.008
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
                    self.target = self.scaling * np.array(
                        [float(v) for v in match.group(1).split(b',')])
                if b'stopj' in buffer:
                    self.target = np.zeros(6)
                buffer = buffer[-512:] if b'speedj' not in buffer else b''
        except OSError:
            pass

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
                values = [float(v) for v in line.strip(b'()').split(b',')]
                self.target = self.scaling * np.array(values)
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
                    if max(numbers) > self.highest_register:
                        connection.sendall(wire.rtde_packet(
                            kind, b'\x00' + b','.join([b'NOT_FOUND'] * len(numbers))))
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
                        self.target = self.scaling * np.array(values[:6])
                        self.vectors += 1
        except (OSError, ConnectionError):
            pass

    def close(self):
        self.running = False
        self.server.close()
        self.rtde_server.close()


def run_bridge(fake, stream, seconds, velocity):
    args = ['--ros-args', '-p', 'robot_ip:=127.0.0.1', '-p', f'port:={fake.port}',
            '-p', 'enable_commands:=true', '-p', f'stream:={stream}',
            '-p', 'host_ip:=127.0.0.1', '-p', f'stream_port:={free_port()}',
            '-p', f'rtde_port:={fake.rtde_port}']
    rclpy.init(args=args)
    bridge = ur_bridge.URBridge()
    driver = rclpy.create_node('bridge_test_driver')
    publisher = driver.create_publisher(Float64MultiArray, '/ur/joint_velocity_command', 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(driver)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        time.sleep(1.0)
        start = fake.q.copy()
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            publisher.publish(Float64MultiArray(data=list(velocity)))
            time.sleep(0.05)
        moved = fake.q - start
        status = bridge.status()
        time.sleep(0.6)                 # stream stops: watchdog must zero the arm
        stopped = fake.target.copy()
        return moved, stopped, bridge.fault, status
    finally:
        executor.shutdown()
        bridge.shutdown()
        bridge.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('stream, highest_register', [
    ('rtde', 47), ('rtde', 23), ('program', 47), ('lines', 47)])
def test_bridge_delivers_velocity_and_stops_when_the_stream_stops(stream, highest_register):
    fake = FakeUR(highest_register=highest_register)
    try:
        moved, stopped, fault, status = run_bridge(fake, stream, 2.0, [0, 0, 0, 0, 0, 0.05])
    finally:
        fake.close()
    assert fault is None and status == 'ready'
    assert moved[5] == pytest.approx(0.10, abs=0.03)
    np.testing.assert_allclose(moved[:5], 0.0, atol=1e-6)
    np.testing.assert_allclose(stopped, 0.0)
    if stream == 'program':
        assert fake.programs.count('stream') == 1 and fake.vectors >= 30
    if stream == 'rtde':
        assert fake.programs.count('rtde') == 1 and fake.vectors >= 30
        # An older controller without registers 24..29 gets the lower set.
        assert fake.rtde_base == (24 if highest_register >= 29 else 18)


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
