#!/usr/bin/env python3
"""Publish a visible RViz marker for a PointStamped goal/frontier."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker


class PointStampedToMarker(Node):
    def __init__(self) -> None:
        super().__init__("pointstamped_to_marker")
        self.declare_parameter("input_topic", "/robot/way_point_coord")
        self.declare_parameter("output_topic", "/robot/frontier_goal_marker")
        self.declare_parameter("scale", 0.35)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(Marker, output_topic, 10)
        self.create_subscription(PointStamped, input_topic, self._on_point, 10)
        self.get_logger().info(f"frontier marker: {input_topic} -> {output_topic}")

    def _on_point(self, msg: PointStamped) -> None:
        scale = float(self.get_parameter("scale").value)
        marker = Marker()
        marker.header = msg.header
        marker.ns = "frontier_goal"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z + 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 5
        self._pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = PointStampedToMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
