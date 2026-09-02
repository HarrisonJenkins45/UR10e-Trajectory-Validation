#!/usr/bin/env python3
"""Publishes the wall/floor obstacles as RViz Markers, since RViz only draws
what's explicitly published to it -- unlike Gazebo, where adding a shape to
the SDF world is enough on its own. Geometry matches validation_core.py's
Cuboid env exactly (floor at Y=1.0, wall at Z=-0.05).
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleMarkerPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_marker_publisher')
        self.pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)
        # Periodic rather than one-shot -- simplest way to guarantee RViz
        # (which may not have subscribed yet at exact node startup) picks
        # these up, without needing a transient-local QoS profile.
        self.timer = self.create_timer(1.0, self.publish_markers)

    def _make_cube(self, marker_id, x, y, z, sx, sy, sz, r, g, b):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'obstacles'
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.6
        return m

    def publish_markers(self):
        floor = self._make_cube(0, 0.0, 1.0, 0.0, 3.0, 0.05, 3.0, 0.1, 0.1, 0.1)
        wall = self._make_cube(1, 0.0, 0.0, -0.05, 3.0, 3.0, 0.05, 0.85, 0.65, 0.0)
        self.pub.publish(MarkerArray(markers=[floor, wall]))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
