#!/usr/bin/env python3


#joint_state_node.py
import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.io import loadmat

class MatTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('joint_state_node')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Joint names matching your URDF (7 joints total: linear base + 6 UR10e joints)
        self.joint_names = [
            'linear_rail_joint',   # Column 8: Linear rail prismatic joint
            'shoulder_pan_joint',  # Column 2
            'shoulder_lift_joint', # Column 3
            'elbow_joint',         # Column 4
            'wrist_1_joint',       # Column 5
            'wrist_2_joint',       # Column 6
            'wrist_3_joint'        # Column 7
        ]

        # 1. Load MAT File
        mat_path = os.path.expanduser('~/ros2_ws/joint_trajectory.mat')
        mat_data = loadmat(mat_path)
        
        # Grab the matrix 
        raw_data = mat_data.get('joint_trajectory', list(mat_data.values())[-1])

        # Ensure matrix is (N, 8)
        if raw_data.shape[0] == 8 and raw_data.shape[1] != 8:
            raw_data = raw_data.T

        # 2. Parse Columns
        self.time_vec = raw_data[:, 0]                     # Col 1: Time steps
        joint_pos_arm = raw_data[:, 1:7]                 # Cols 2-7: Arm joint position
        rail_pos = raw_data[:, 7:8]                      # Col 8: Linear rail position
        
        # Reorder velocities to match joint_names order [rail, arm_1..6]
        self.positions = np.hstack((rail_pos, joint_pos_arm))

        # Compute dt vector: dt[i] = t[i] - t[i-1]
        dt = np.diff(self.time_vec, prepend=self.time_vec[0])
        


        self.get_logger().info(f"Loaded {len(self.positions)} trajectory waypoints.")

        # 3. Timer Setup based on actual MAT sampling time
        avg_dt = float(np.mean(dt[1:])) if len(dt) > 1 else 0.02
        self.current_idx = 0
        self.timer = self.create_timer(avg_dt, self.timer_callback)

    def timer_callback(self):
        if self.current_idx >= len(self.positions):
            self.current_idx = 0  # Loop back to beginning

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions[self.current_idx].tolist()

        self.publisher_.publish(msg)

        if self.current_idx % 20 == 0:
            self.get_logger().info(f"Step {self.current_idx}/{len(self.positions)} | Rail Pos: {msg.position[0]:.3f} m")

        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = MatTrajectoryPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
