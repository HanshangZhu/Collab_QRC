#!/usr/bin/env python3
"""Publish a persistent RViz marker for a PoseStamped goal."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker


class PoseStampedToMarker(Node):
    def __init__(self) -> None:
        super().__init__("posestamped_to_marker")
        self.declare_parameter("input_topic", "/goal_pose")
        self.declare_parameter("output_topic", "/robot/manual_goal_marker")
        self.declare_parameter("scale", 0.45)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._scale = float(self.get_parameter("scale").value)
        self._pub = self.create_publisher(Marker, output_topic, 10)
        self.create_subscription(PoseStamped, input_topic, self._on_pose, 10)
        self.get_logger().info(f"manual goal marker: {input_topic} -> {output_topic}")

    def _on_pose(self, msg: PoseStamped) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.ns = "manual_goal"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.pose.position.z += 0.25
        marker.scale.x = self._scale
        marker.scale.y = self._scale * 0.22
        marker.scale.z = self._scale * 0.22
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self._pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = PoseStampedToMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
