"""rail_bridge against a fake Parker controller that speaks the real wire protocol.

The fake answers on TCP like the rig (echo, value, SYS> prompt), streams
320-byte fast-status packets over UDP to whoever subscribed, and integrates
jog motion with the configured acceleration and deceleration. It can be told
to misbehave the ways the rail has: a jog left running by a previous process,
a JOG OFF that is not acted on, a jog started by another client, a slow
deceleration. It checks the PC side end to end; it cannot check how the real
controller orders or buffers commands.
"""

import contextlib
import math
import socket
import struct
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float64  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from ur10e_trajectory_pkg import hardware_protocol as wire  # noqa: E402
from ur10e_trajectory_pkg import rail_bridge  # noqa: E402


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class FakeParker:
    """Units are mm (units_per_mm = 1), as the bridge is configured below."""

    def __init__(self, position_mm=1000.0, jogging=0.0, decel=10.0, homed=True,
                 ignore_stop=False, home_switch_mm=None):
        self.position = position_mm
        self.velocity = 0.0
        self.target = jogging
        self.jog_speed = abs(jogging)
        self.jog_active = jogging != 0
        self.drive = jogging != 0
        self.accel, self.decel = 10.0, decel
        self.homed = homed
        self.homing = False
        self.home_switch = home_switch_mm
        self.ignore_stop = ignore_stop
        self.reversed_while_moving = False
        # Like the rig: fast-status packets read all zero until FSTAT ON.
        self.fstat_on = False
        self.lines = []
        self.lock = threading.Lock()
        self.running = True
        self.subscriber = None
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(8)
        self.tcp_port = self.server.getsockname()[1]
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0))
        self.udp.settimeout(0.05)
        self.udp_port = self.udp.getsockname()[1]
        for target in (self.accept, self.serve_udp, self.simulate):
            threading.Thread(target=target, daemon=True).start()

    # --- motion ----------------------------------------------------------------

    def start_jog(self, velocity):
        """What another client's JOG VEL + JOG FWD/REV would do."""
        with self.lock:
            self.drive = True
            self.jog_speed = abs(velocity)
            self.target, self.jog_active = velocity, True

    def simulate(self):
        dt = 0.01
        while self.running:
            with self.lock:
                if not self.drive:
                    self.velocity = self.target = 0.0
                    self.jog_active = self.homing = False
                else:
                    if self.homing:
                        self.target = -self.jog_speed
                    error = self.target - self.velocity
                    speeding_up = abs(self.target) > abs(self.velocity) and \
                        self.target * self.velocity >= 0
                    rate = (self.accel if speeding_up else self.decel) * dt
                    self.velocity += max(-rate, min(rate, error))
                    self.position += self.velocity * dt
                    if (self.homing and self.home_switch is not None
                            and self.position <= self.home_switch):
                        self.position, self.velocity, self.target = 0.0, 0.0, 0.0
                        self.homing, self.homed, self.jog_active = False, True, False
                    if self.jog_active and self.target == 0 and self.velocity == 0:
                        self.jog_active = False
                packet = bytearray(320)
                if self.fstat_on:
                    struct.pack_into('>i', packet, 0,
                                     int(round(self.position * wire.RAIL_COUNTS_PER_MM)))
                    struct.pack_into('>f', packet, 32, self.velocity)
                subscriber = self.subscriber
            if subscriber is not None:
                try:
                    self.udp.sendto(bytes(packet), subscriber)
                except OSError:
                    pass
            time.sleep(dt)

    # --- UDP -------------------------------------------------------------------

    def serve_udp(self):
        while self.running:
            try:
                data, peer = self.udp.recvfrom(64)
            except (socket.timeout, OSError):
                continue
            if data == wire.RAIL_SUBSCRIBE:
                self.subscriber = peer
            elif data == wire.RAIL_UNSUBSCRIBE:
                self.subscriber = None

    # --- TCP -------------------------------------------------------------------

    def accept(self):
        while self.running:
            try:
                connection, _ = self.server.accept()
            except OSError:
                return
            threading.Thread(target=self.serve, args=(connection,), daemon=True).start()

    def serve(self, connection):
        buffer = b''
        try:
            while self.running:
                data = connection.recv(4096)
                if not data:
                    return
                buffer += data
                while b'\r' in buffer:
                    line, buffer = buffer.split(b'\r', 1)
                    text = line.decode('ascii').strip()
                    if text:
                        connection.sendall(self.execute(text))
        except OSError:
            pass
        finally:
            connection.close()

    def execute(self, text):
        values = []
        with self.lock:
            for command in text.split(':'):
                self.lines.append(command)
                value = self.apply(command)
                if value is not None:
                    values.append(value)
        reply = text + '\r\n' + ''.join(f'{v}\r\n' for v in values) + 'SYS>'
        return reply.encode('ascii')

    def apply(self, command):
        words = command.split()
        if command == 'PRINT BIT 16134':
            return -1 if self.homed else 0
        if command == 'PRINT BIT 792':
            return -1 if self.jog_active else 0
        if command == 'PRINT P12349':
            return self.accel
        if command == 'PRINT P12350':
            return self.decel
        if command == 'VER':
            return 'IPA Drive 4.46 (fake)'
        if words[:3] == ['AXIS0', 'JOG', 'VEL']:
            self.jog_speed = float(words[3])
        elif command in ('AXIS0 JOG FWD', 'AXIS0 JOG REV') and self.drive:
            sign = 1.0 if command.endswith('FWD') else -1.0
            if self.velocity * sign < 0 or self.target * sign < 0:
                self.reversed_while_moving = True
            self.target, self.jog_active = sign * self.jog_speed, True
        elif command == 'AXIS0 JOG OFF':
            if not self.ignore_stop:
                self.target, self.homing = 0.0, False
        elif words[:3] == ['AXIS0', 'JOG', 'HOME'] and self.drive:
            self.homing, self.homed, self.jog_active = True, False, True
        elif command == 'FSTAT ON':
            self.fstat_on = True
        elif command == 'AXIS0 DRIVE ON':
            self.drive = True
        elif command == 'AXIS0 DRIVE OFF':
            self.drive = False
        return None

    def sent(self, command):
        with self.lock:
            return self.lines.count(command)

    def close(self):
        self.running = False
        self.server.close()
        self.udp.close()


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
        publisher.publish(Float64(data=velocity))
        time.sleep(0.05)


@contextlib.contextmanager
def bridge_for(fake, enable=True, **extra):
    params = {'host': '127.0.0.1', 'command_port': fake.tcp_port,
              'feedback_port': free_udp_port(), 'controller_feedback_port': fake.udp_port,
              'enable_commands': enable, **extra}
    args = ['--ros-args']
    for name, value in params.items():
        args += ['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}']
    rclpy.init(args=args)
    bridge = rail_bridge.RailBridge()
    # The executor's node name: the only publisher the bridge accepts.
    driver = rclpy.create_node('plan_executor')
    publisher = driver.create_publisher(Float64, '/rail/velocity_command', 10)
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
    return wait_until(lambda: bridge.status() == 'ready', 5.0)


def test_a_jog_left_running_is_stopped_first_then_the_drive_switched_on():
    fake = FakeParker(jogging=8.0)
    with bridge_for(fake) as (bridge, _):
        assert ready(bridge), bridge.status()
        assert fake.lines[0] == 'AXIS0 JOG OFF'
        assert fake.sent('AXIS0 DRIVE ON') == 1
        assert fake.velocity == 0.0 and not fake.jog_active
        assert bridge.fault is None and bridge.state == 'idle'


def test_commands_jog_the_rail_and_zero_ends_in_a_verified_stop():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        start = fake.position
        stream(publisher, 0.005, 1.0)
        assert bridge.state == 'jogging'
        stream(publisher, 0.0, 0.2)
        assert wait_until(lambda: bridge.state == 'idle', 3.0)
        # 0.5 s ramp to 5 mm/s, cruise, then 0.5 s of deceleration: ~5 mm.
        assert fake.position - start == pytest.approx(5.0, abs=1.5)
        assert fake.velocity == 0.0 and bridge.fault is None
        assert bridge.status() == 'ready'


def test_a_stalled_command_stream_latches_a_fault_and_stops():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, 0.005, 0.8)
        assert wait_until(lambda: bridge.fault is not None, 1.0)
        assert 'no rail command' in bridge.fault
        assert wait_until(lambda: bridge.state == 'idle', 3.0)
        assert fake.velocity == 0.0
        stream(publisher, 0.005, 0.3)       # a latched fault refuses motion
        assert fake.velocity == 0.0 and bridge.status().startswith('fault')


def test_motion_nobody_commanded_is_stopped_and_latched():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, _):
        assert ready(bridge)
        jog_offs = fake.sent('AXIS0 JOG OFF')
        fake.start_jog(4.0)                 # another client, or a jog that survived
        assert wait_until(lambda: bridge.fault is not None, 2.0)
        assert fake.sent('AXIS0 JOG OFF') > jog_offs
        assert wait_until(lambda: fake.velocity == 0.0 and not fake.jog_active, 3.0)
        assert bridge.status().startswith('fault')


def test_a_jog_off_that_is_not_acted_on_escalates_to_drive_off():
    fake = FakeParker(ignore_stop=True)
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, 0.005, 0.8)
        publisher.publish(Float64(data=0.0))
        assert wait_until(lambda: fake.sent('AXIS0 DRIVE OFF') >= 1, 5.0)
        assert fake.sent('AXIS0 JOG OFF') >= 5      # resent until the deadline
        assert 'DRIVE OFF' in bridge.fault
        assert wait_until(lambda: fake.velocity == 0.0, 1.0)


def test_a_reversal_stops_the_rail_before_jogging_the_other_way():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, 0.005, 0.8)
        turned = fake.position
        stream(publisher, -0.005, 2.0)
        assert not fake.reversed_while_moving
        assert fake.position < turned and fake.velocity < 0
        assert bridge.fault is None


def test_another_publisher_on_the_command_topic_blocks_motion():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stranger = rclpy.create_node('forgotten_topic_pub')
        stranger.create_publisher(Float64, '/rail/velocity_command', 10)
        try:
            assert wait_until(lambda: bridge.check_command_sources() != [], 2.0)
            stream(publisher, 0.005, 0.5)
            assert fake.sent('AXIS0 JOG FWD') == 0
            assert 'forgotten_topic_pub' in bridge.status()
        finally:
            stranger.destroy_node()


def test_a_deceleration_too_slow_to_stop_in_time_is_refused():
    fake = FakeParker(decel=0.1)
    with bridge_for(fake) as (bridge, publisher):
        assert wait_until(lambda: bridge.fault is not None, 5.0)
        assert 'jog deceleration 0.1' in bridge.fault
        assert fake.sent('AXIS0 DRIVE ON') == 0
        stream(publisher, 0.005, 0.3)
        assert fake.sent('AXIS0 JOG FWD') == 0


def test_shadow_mode_stops_but_never_moves_or_powers_the_drive():
    fake = FakeParker(jogging=5.0)
    with bridge_for(fake, enable=False) as (bridge, publisher):
        assert wait_until(lambda: bridge.preflight_done, 5.0)
        assert fake.lines[0] == 'AXIS0 JOG OFF' and fake.velocity == 0.0
        stream(publisher, 0.005, 0.5)
        assert fake.sent('AXIS0 JOG FWD') == 0 and fake.sent('AXIS0 DRIVE ON') == 0
    assert fake.sent('AXIS0 DRIVE OFF') == 0


def test_homing_finishes_with_a_verified_stop_before_the_flag_is_published():
    fake = FakeParker(position_mm=450.0, homed=False, home_switch_mm=420.0)
    with bridge_for(fake, homing_speed=0.025) as (bridge, _):
        assert ready(bridge) and bridge.homed is False
        response = bridge.on_home(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert wait_until(lambda: bridge.homed is True, 8.0)
        assert bridge.state == 'idle' and fake.homed and fake.position == 0.0
        assert fake.lines.index('AXIS0 JOG HOME -1') < len(fake.lines) - 1 - \
            fake.lines[::-1].index('AXIS0 JOG OFF')


def test_shutdown_stops_the_rail_and_switches_the_drive_off():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, 0.005, 0.6)
        bridge.shutdown()
        assert fake.velocity == 0.0 and not fake.jog_active
        assert fake.sent('AXIS0 DRIVE OFF') == 1 and not fake.drive
        assert math.isfinite(fake.position)


def test_zero_packets_before_fstat_are_not_taken_for_position_or_motion():
    # The rig: zeros until FSTAT ON, then the real count far from zero.
    fake = FakeParker(position_mm=-27500.0)
    with bridge_for(fake) as (bridge, _):
        assert ready(bridge), bridge.status()
        assert bridge.fault is None
        assert bridge.feedback()[0] == pytest.approx(-27.5, abs=0.001)


def test_a_feedback_jump_while_jogging_faults():
    fake = FakeParker()
    with bridge_for(fake) as (bridge, publisher):
        assert ready(bridge)
        stream(publisher, 0.005, 0.6)
        with fake.lock:
            fake.position += 500.0          # 0.5 m in one packet
        stream(publisher, 0.005, 0.2)
        assert bridge.fault is not None and 'jumped' in bridge.fault
