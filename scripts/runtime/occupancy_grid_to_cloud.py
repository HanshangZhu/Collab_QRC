#!/usr/bin/env python3
"""Convert an OccupancyGrid into a sparse PointCloud2 for RViz debugging.

RViz2's Map display can fail on some driver/runtime combinations. This helper
keeps the live grid visible by publishing known cells as a flat point cloud.
"""

from __future__ import annotations

import math
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class OccupancyGridToCloud(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_to_cloud")
        self.declare_parameter("input_topic", "/robot/traversability_grid")
        self.declare_parameter("output_topic", "/robot/traversability_cloud")
        self.declare_parameter("stride", 2)
        self.declare_parameter("z", 0.04)
        self.declare_parameter("max_points", 60000)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        grid_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(PointCloud2, output_topic, cloud_qos)
        self.create_subscription(OccupancyGrid, input_topic, self._on_grid, grid_qos)
        self.get_logger().info(f"grid cloud: {input_topic} -> {output_topic}")

    def _on_grid(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return

        stride = max(1, int(self.get_parameter("stride").value))
        max_points = max(1, int(self.get_parameter("max_points").value))
        z = float(self.get_parameter("z").value)
        points = list(self._iter_points(msg, stride, z, max_points))
        if not points:
            return

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(msg.header, fields, points)
        self._pub.publish(cloud)

    @staticmethod
    def _iter_points(
        msg: OccupancyGrid,
        stride: int,
        z: float,
        max_points: int,
    ) -> Iterable[tuple[float, float, float, float]]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        emitted = 0
        for gy in range(0, height, stride):
            base = gy * width
            y = oy + (gy + 0.5) * res
            for gx in range(0, width, stride):
                occ = int(msg.data[base + gx])
                if occ < 0:
                    continue
                x = ox + (gx + 0.5) * res
                intensity = float(min(100, max(0, occ)))
                yield (x, y, z, intensity)
                emitted += 1
                if emitted >= max_points:
                    return


def main() -> None:
    rclpy.init()
    node = OccupancyGridToCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
