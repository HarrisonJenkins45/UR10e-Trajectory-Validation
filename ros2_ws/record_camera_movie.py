#!/usr/bin/env python3
"""Continuously record frames from the bridged Gazebo camera topic to an mp4.

IMPORTANT: stop this with Ctrl+C, not by killing the process (e.g. closing
the terminal or `kill -9`). The mp4 container format needs to be finalized
on close, and Ctrl+C triggers that cleanup; a hard kill will likely leave
you with an unplayable file.

Usage (inside the container, after the camera bridge is running):
    python3 record_camera_video.py [output_path] [fps]

Defaults: /root/ros2_ws/robot_trajectory_gazebo.mp4 at 30 fps
(30 fps matches both the camera sensor's <update_rate> in empty_camera.sdf
and the framerate used in the MATLAB script, for consistency).
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoRecorder(Node):
    def __init__(self, output_path, fps):
        super().__init__('video_recorder')
        self.bridge = CvBridge()
        self.writer = None
        self.output_path = output_path
        self.fps = fps
        self.frame_count = 0
        self.sub = self.create_subscription(Image, '/camera', self.callback, 10)
        self.get_logger().info(f'Recording to {output_path} at {fps} fps. Ctrl+C to stop and finalize.')

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.writer is None:
            h, w = cv_image.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
            self.get_logger().info(f'Opened video writer at {w}x{h}')

        self.writer.write(cv_image)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'{self.frame_count} frames recorded')

    def cleanup(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f'Finalized {self.output_path} ({self.frame_count} frames)')
        else:
            self.get_logger().warn('No frames were ever received -- nothing written. '
                                    'Is the camera bridge (ros_gz_bridge) still running?')


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/robot_trajectory_gazebo.mp4'
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

    rclpy.init()
    node = VideoRecorder(output_path, fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
