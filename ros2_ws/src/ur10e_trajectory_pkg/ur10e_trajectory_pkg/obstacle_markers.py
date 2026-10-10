#!/usr/bin/env python3
"""Publishes the wall/floor obstacles as RViz Markers, since RViz only draws
what's explicitly published to it -- unlike Gazebo, where adding a shape to
the SDF world is enough on its own. Geometry matches validation_core.py's
Cuboid env exactly.

The rig's rail is bolted to a wall, so the model's frame is the wall's:
+x along the rail, +y up, +z out of the wall. The wall is the plane thin in
Z just behind the rail, at Z = -0.05; the floor is the plane thin in Y,
1 m below the rail at Y = -1.0, within the arm's reach. (An earlier model read this frame as a
floor-mounted rail: a "floor" at Z = -0.05, which was really this wall, and a
"wall" at Y = +1.0, which was a ceiling the rig does not have.)
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
        # The wall the rail is bolted to: thin in Z at Z = -0.05, from the
        # floor (Y = -1) to 1.5 m above the rail. Dark grey.
        wall = self._make_cube(0, 1.5, 0.25, -0.05, 6.0, 2.5, 0.05, 0.1, 0.1, 0.1)
        # The floor 1 m below the rail: thin in Y at Y = -1.0, 3 m out from
        # the wall, within the arm's reach. Gold.
        floor = self._make_cube(1, 1.5, -1.0, 1.5, 6.0, 0.05, 3.0, 0.85, 0.65, 0.0)
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
