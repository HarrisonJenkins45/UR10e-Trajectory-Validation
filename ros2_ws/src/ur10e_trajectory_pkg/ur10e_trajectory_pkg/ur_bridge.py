#!/usr/bin/env python3
"""ROS <-> UR10e port 30003, replacing the Simulink tcpUR10eBlock and UR10eSend.

State:    reads realtime packets from 30003 and publishes the six arm joints on
          /ur/joint_states (JointState, position and velocity).
Command:  forwards /ur/joint_velocity_command (Float64MultiArray, six rad/s)
          to 30003 as a URScript speedj line, exactly as the Simulink model
          streamed it.

Safety behaviour, none of which the Simulink model had:
  * enable_commands is false by default: the node then reads state and logs
    what it would send, without opening a command connection (shadow mode).
  * speedj_time is short, so the controller itself stops the arm when the
    stream stops, even if this process dies.
  * a command older than command_timeout triggers stopj.
  * a malformed or over-speed command sends stopj and latches a fault until
    ~/reset_fault is called.

The robot must be in Remote Control mode to accept scripts from the network.
Each script sent to 30003 replaces the running program, so do not run this
alongside another program (pendant or ur_robot_driver) that must keep running.
"""

import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from ur10e_trajectory_pkg import hardware_protocol as wire
from ur10e_trajectory_pkg.configurations import JOINT_NAMES
from ur10e_trajectory_pkg.ros_params import declare

ARM_JOINT_NAMES = list(JOINT_NAMES[1:])


def _receive_exactly(sock, count):
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError('UR closed the realtime connection')
        data.extend(chunk)
    return bytes(data)


class URBridge(Node):

    def __init__(self):
        super().__init__('ur_bridge')
        self.robot_ip = declare(self, 'robot_ip', '192.168.7.8')
        self.port = declare(self, 'port', wire.UR_REALTIME_PORT)
        self.enable_commands = declare(self, 'enable_commands', False)
        self.acceleration = declare(self, 'acceleration', 1.0)
        self.stop_deceleration = declare(self, 'stop_deceleration', 2.0)
        self.speedj_time = declare(self, 'speedj_time', 0.2)
        self.command_timeout = declare(self, 'command_timeout', 0.25)
        self.max_joint_speed = declare(self, 'max_joint_speed', 0.5)
        self.publish_hz = declare(self, 'publish_hz', 125.0)

        self.state_pub = self.create_publisher(JointState, '/ur/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/ur/joint_velocity_command',
                                 self.on_command, 10)
        self.create_service(Trigger, '~/reset_fault', self.on_reset)
        self.create_timer(0.02, self.watchdog)

        self.command_sock = None
        self.moving = False
        self.fault = None
        self.last_command = 0.0
        self.running = True
        self.reader = threading.Thread(target=self.read_state, daemon=True)
        self.reader.start()
        mode = 'COMMANDS ENABLED' if self.enable_commands else 'shadow mode (no commands sent)'
        self.get_logger().info(f'UR bridge to {self.robot_ip}:{self.port}, {mode}')

    # --- state -------------------------------------------------------------

    def read_state(self):
        period = 1.0 / self.publish_hz
        while self.running:
            try:
                with socket.create_connection((self.robot_ip, self.port), timeout=2.0) as sock:
                    sock.settimeout(1.0)
                    self.get_logger().info('Receiving UR realtime state')
                    last_publish = 0.0
                    while self.running:
                        header = _receive_exactly(sock, 4)
                        length = wire.ur_packet_length(header)
                        packet = header + _receive_exactly(sock, length - 4)
                        now = time.monotonic()
                        if now - last_publish < period:
                            continue
                        last_publish = now
                        state = wire.parse_ur_realtime(packet)
                        message = JointState()
                        message.header.stamp = self.get_clock().now().to_msg()
                        message.name = ARM_JOINT_NAMES
                        message.position = state['q'].tolist()
                        message.velocity = state['qd'].tolist()
                        self.state_pub.publish(message)
            except (OSError, ValueError, ConnectionError) as error:
                if self.running:
                    self.get_logger().warn(f'UR state connection: {error}; retrying',
                                           throttle_duration_sec=5.0)
                    time.sleep(1.0)

    # --- commands ----------------------------------------------------------

    def send(self, line):
        """Send one script line; return False, never raise, on failure."""
        if not self.enable_commands:
            self.get_logger().info(f'[shadow] {line.decode().strip()}',
                                   throttle_duration_sec=1.0)
            return True
        try:
            if self.command_sock is None:
                sock = socket.create_connection((self.robot_ip, self.port), timeout=1.0)
                # Port 30003 streams state on every connection, this one too.
                # Drain it, or the controller's send buffer backs up.
                threading.Thread(target=self.drain, args=(sock,), daemon=True).start()
                self.command_sock = sock
            self.command_sock.sendall(line)
            return True
        except OSError as error:
            self.close_command_socket()
            self.get_logger().error(f'UR command send failed: {error}')
            return False

    @staticmethod
    def drain(sock):
        try:
            while sock.recv(65536):
                pass
        except OSError:
            pass

    def stop(self):
        self.moving = False
        if not self.send(wire.stopj_command(self.stop_deceleration)):
            self.get_logger().error('STOP DELIVERY FAILED: use the pendant stop. '
                                    f'speedj times out within {self.speedj_time} s.')

    def trip(self, reason):
        if self.fault is None:
            self.fault = reason
            self.get_logger().error(f'UR bridge fault: {reason}')
        self.stop()

    def on_command(self, message):
        if self.fault:
            self.get_logger().warn(f'ignoring command, fault latched: {self.fault}',
                                   throttle_duration_sec=2.0)
            return
        velocities = np.asarray(message.data, dtype=float)
        if velocities.shape != (6,) or not np.all(np.isfinite(velocities)):
            self.trip(f'malformed arm command {list(message.data)}')
            return
        if np.max(np.abs(velocities)) > self.max_joint_speed:
            self.trip(f'arm command {np.round(velocities, 3).tolist()} exceeds '
                      f'{self.max_joint_speed} rad/s')
            return
        self.last_command = time.monotonic()
        if not np.any(velocities):
            if self.moving:
                self.stop()
            return
        self.moving = True
        if not self.send(wire.speedj_command(velocities, self.acceleration,
                                             self.speedj_time)):
            self.trip('speedj send failed')

    def watchdog(self):
        if self.moving and time.monotonic() - self.last_command > self.command_timeout:
            self.get_logger().warn('arm command stream stopped; sending stopj')
            self.stop()

    def on_reset(self, request, response):
        self.fault = None
        response.success = True
        response.message = 'UR bridge fault cleared'
        return response

    def close_command_socket(self):
        if self.command_sock is not None:
            try:
                self.command_sock.close()
            finally:
                self.command_sock = None

    def shutdown(self):
        self.running = False
        try:
            if self.moving or self.command_sock is not None:
                self.stop()
        finally:
            self.close_command_socket()


def main(args=None):
    rclpy.init(args=args)
    node = URBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
