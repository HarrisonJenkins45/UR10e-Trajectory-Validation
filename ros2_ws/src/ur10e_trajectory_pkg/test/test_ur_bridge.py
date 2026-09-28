"""ur_bridge against a fake UR controller that speaks the real wire protocol.

The fake serves 1116-byte realtime packets on every connection, runs the
streaming program's side of the handshake (connect back, read velocity
vectors) when it receives that program, executes one-line speedj programs,
and integrates the resulting velocity into its joint positions. It checks the
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

from ur10e_trajectory_pkg import ur_bridge  # noqa: E402

PACKET_DOUBLES = 139


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class FakeUR:
    def __init__(self, scaling=1.0):
        self.scaling = scaling
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
        threading.Thread(target=self.accept, daemon=True).start()
        threading.Thread(target=self.integrate, daemon=True).start()

    def packet(self):
        values = np.zeros(PACKET_DOUBLES)
        values[0] = time.monotonic()
        values[7:13] = self.target
        values[31:37] = self.q
        values[37:43] = self.target
        values[94], values[101] = 7, 1          # RUNNING, NORMAL
        values[117], values[131] = self.scaling, 2
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
                if b'def ros_speedj_stream' in buffer and b'\nend\n' in buffer:
                    self.programs.append('stream')
                    host, port = re.search(rb'socket_open\("([\d.]+)", (\d+)', buffer).groups()
                    threading.Thread(target=self.stream, args=(host.decode(), int(port)),
                                     daemon=True).start()
                    buffer = b''
                for match in re.finditer(rb'speedj\(\[([^\]]*)\]', buffer):
                    self.programs.append('line')
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

    def close(self):
        self.running = False
        self.server.close()


def run_bridge(fake, stream, seconds, velocity):
    args = ['--ros-args', '-p', 'robot_ip:=127.0.0.1', '-p', f'port:={fake.port}',
            '-p', 'enable_commands:=true', '-p', f'stream:={stream}',
            '-p', 'host_ip:=127.0.0.1', '-p', f'stream_port:={free_port()}']
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
        time.sleep(0.6)                 # stream stops: watchdog must zero the arm
        stopped = fake.target.copy()
        return moved, stopped, bridge.fault
    finally:
        executor.shutdown()
        bridge.shutdown()
        bridge.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('stream', ['program', 'lines'])
def test_bridge_delivers_velocity_and_stops_when_the_stream_stops(stream):
    fake = FakeUR()
    try:
        moved, stopped, fault = run_bridge(fake, stream, 2.0, [0, 0, 0, 0, 0, 0.05])
    finally:
        fake.close()
    assert fault is None
    assert moved[5] == pytest.approx(0.10, abs=0.03)
    np.testing.assert_allclose(moved[:5], 0.0, atol=1e-6)
    np.testing.assert_allclose(stopped, 0.0)
    if stream == 'program':
        assert fake.programs.count('stream') == 1 and fake.vectors >= 30


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
