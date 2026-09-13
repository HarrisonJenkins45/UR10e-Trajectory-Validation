#!/usr/bin/env python3
"""Save a single frame from the bridged Gazebo camera topic, then exit.
Quick sanity check: run this to confirm the camera actually sees the robot
before wiring up continuous recording.

Usage (inside the container, after the bridge is running):
    python3 save_camera_frame.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


OUTPUT_PATH = '/root/ros2_ws/camera_snapshot.png'


class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        self.bridge = CvBridge()
        self.saved = False
        self.sub = self.create_subscription(Image, '/camera', self.callback, 10)
        self.get_logger().info('Waiting for a frame on /camera ...')

    def callback(self, msg):
        if self.saved:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(OUTPUT_PATH, cv_image)
        self.get_logger().info(f'Saved frame to {OUTPUT_PATH}')
        self.saved = True
        rclpy.shutdown()


def main():
    rclpy.init()
    node = FrameSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
