#!/usr/bin/env python3
"""Merge /ur/joint_states and /rail/joint_state into the robot's /joint_states.

This is the measured seven-joint state that robot_state_publisher, RViz and
the plan executor consume, like the Simulink Gamma/dGamma mux. Nothing is
published unless both halves are fresh, so a lost device freezes the RViz
model rather than showing a stale pose as live.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ur10e_trajectory_pkg.configurations import JOINT_NAMES
from ur10e_trajectory_pkg.ros_params import declare


class JointStateMerger(Node):

    def __init__(self):
        super().__init__('joint_state_merger')
        self.timeout = declare(self, 'timeout', 0.25)
        rate_hz = declare(self, 'rate_hz', 50.0)
        self.latest = {}
        self.create_subscription(JointState, '/ur/joint_states', self.on_state, 10)
        self.create_subscription(JointState, '/rail/joint_state', self.on_state, 10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / rate_hz, self.publish)

    def on_state(self, message):
        now = time.monotonic()
        velocities = list(message.velocity) or [0.0] * len(message.name)
        for name, position, velocity in zip(message.name, message.position, velocities):
            self.latest[name] = (position, velocity, now)

    def publish(self):
        now = time.monotonic()
        entries = [self.latest.get(name) for name in JOINT_NAMES]
        missing = [name for name, entry in zip(JOINT_NAMES, entries)
                   if entry is None or now - entry[2] > self.timeout]
        if missing:
            self.get_logger().warn(f'no fresh state for {missing}', throttle_duration_sec=5.0)
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [entry[0] for entry in entries]
        message.velocity = [entry[1] for entry in entries]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A signal can shut ROS down mid-spin; only real errors propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
