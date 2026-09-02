#!/usr/bin/env python3
"""Bridges /joint_states (as published by Validate_trajServer.py) into the
per-joint command topics that Gazebo's JointPositionController plugins
listen on. Without this, /joint_states only reaches RViz/TF -- it never
reaches Gazebo's physics.

IMPORTANT: JointPositionController plugins have NO target position at all
until their first cmd_pos message arrives -- until then they're
uncontrolled (free to drift/oscillate under gravity/residual dynamics).
Since nothing publishes /joint_states until a trajectory request actually
succeeds, this node publishes an explicit home-hold command on startup
(and for a short window afterward, to survive any early message drops
before Gazebo's plugins are fully up) so every joint has a real setpoint
to hold from the first simulation tick.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Matches qHome in the MATLAB script / Validate_trajServer.py's default
# seed: rail=0, then [0, -135, 90, -90, 0, 0] deg for the 6 arm joints.
import math
HOME_POSITION_RAD = [0.0, 0.0, math.radians(-135.0), math.radians(90.0),
                      math.radians(-90.0), 0.0, 0.0]


class JointStateToGazeboBridge(Node):
    def __init__(self):
        super().__init__('joint_state_to_gazebo_bridge')

        # Must match the joint names used in Validate_trajServer.py / the
        # URDF's JointPositionController plugin blocks exactly.
        self.joint_names = [
            'linear_rail_joint',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.cmd_publishers = {
            name: self.create_publisher(Float64, f'/cmd/{name}', 10)
            for name in self.joint_names
        }

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10
        )
        self.get_logger().info('Bridging /joint_states -> per-joint /cmd/<joint> topics.')

        # Publish the home-hold command a handful of times over ~2s, since
        # Gazebo's plugins may not be fully subscribed yet at the exact
        # instant this node starts -- a single publish could be missed.
        self._home_hold_count = 0
        self.publish_home_hold()
        self._home_hold_timer = self.create_timer(0.2, self.publish_home_hold)

    def publish_home_hold(self):
        for name, pos in zip(self.joint_names, HOME_POSITION_RAD):
            cmd_msg = Float64()
            cmd_msg.data = float(pos)
            self.cmd_publishers[name].publish(cmd_msg)
        self._home_hold_count += 1
        if self._home_hold_count == 1:
            self.get_logger().info('Publishing home-hold command so joints have a setpoint before any trajectory arrives.')
        if self._home_hold_count >= 10:  # ~2s at 0.2s period
            self._home_hold_timer.cancel()

    def callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        for name in self.joint_names:
            if name not in joint_map:
                continue
            cmd_msg = Float64()
            cmd_msg.data = float(joint_map[name])
            self.cmd_publishers[name].publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToGazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
