#!/usr/bin/env python3
"""Bridges /joint_states (as published by Validate_trajServer.py) into the
per-joint command topics that Gazebo's JointPositionController plugins
listen on. Without this, /joint_states only reaches RViz/TF -- it never
reaches Gazebo's physics.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


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
