#!/usr/bin/env python3
"""Publishes the wall/floor obstacles as RViz Markers, since RViz only draws
what's explicitly published to it -- unlike Gazebo, where adding a shape to
the SDF world is enough on its own. Geometry matches validation_core.py's
Cuboid env exactly.

NAMES FOLLOW GEOMETRY, not the other way round. The wall is the vertical
plane (thin in Y, spanning X-Z) standing at Y = +1.0. The floor is the
horizontal plane (thin in Z, spanning X-Y) lying just under the rig at
Z = -0.05. These two were previously swapped here and in validation_core,
which made the gold horizontal floor read as a wall in RViz.
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
        # 6 m long, centred at X = +1.5, so both planes span -1.5 -> 4.5.
        #
        # They must cover the REACHABLE workspace, not the rail's travel. The
        # rail spans 0 -> 3, but the arm overhangs each end by its own 1.3 m
        # reach, so the robot can occupy -1.3 -> 4.3. When these were 3 m
        # long the arm could dip below floor level within 1.3 m of either end
        # with no floor there to hit, and the collision checker had nothing to
        # report.
        # Vertical plane at Y = +1.0, thin in Y -- the wall. Dark grey.
        wall = self._make_cube(0, 1.5, 1.0, 0.0, 6.0, 0.05, 3.0, 0.1, 0.1, 0.1)
        # Horizontal plane at Z = -0.05, thin in Z -- the floor. Gold.
        floor = self._make_cube(1, 1.5, 0.0, -0.05, 6.0, 3.0, 0.05, 0.85, 0.65, 0.0)
        self.pub.publish(MarkerArray(markers=[wall, floor]))


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
