#!/usr/bin/env python3
"""
Render dual-robot explored coverage areas in RViz from OccupancyGrid maps.

- Robot A map cells are shown as transparent red cubes.
- Robot B map cells are shown as transparent green cubes.
- Robot pose markers and text labels are published for go2_1 and go2_2.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class DualMapCoverageVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("dual_map_coverage_visualizer")

        self.declare_parameter("robot_a_map_topic", "/robot_a/map")
        self.declare_parameter("robot_b_map_topic", "/robot_b/map")
        self.declare_parameter("marker_topic", "/dual_robot/coverage_markers")
        self.declare_parameter("robot_a_odom_topic", "/robot_a/odom/ground_truth")
        self.declare_parameter("robot_b_odom_topic", "/robot_b/odom/ground_truth")
        # Empty string means "follow each map frame"; this avoids fixed-frame mismatches.
        self.declare_parameter("marker_frame", "")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("min_map_value", 0)
        self.declare_parameter("cell_stride", 1)
        self.declare_parameter("z_offset", 0.02)
        self.declare_parameter("robot_a_alpha", 0.22)
        self.declare_parameter("robot_b_alpha", 0.22)
        self.declare_parameter("robot_marker_size", 0.22)
        self.declare_parameter("robot_marker_alpha", 0.9)
        self.declare_parameter("robot_label_scale", 0.35)
        self.declare_parameter("robot_label_z_offset", 0.45)

        self.robot_a_map_topic = str(self.get_parameter("robot_a_map_topic").value)
        self.robot_b_map_topic = str(self.get_parameter("robot_b_map_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.robot_a_odom_topic = str(self.get_parameter("robot_a_odom_topic").value)
        self.robot_b_odom_topic = str(self.get_parameter("robot_b_odom_topic").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value).strip()
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.min_map_value = int(self.get_parameter("min_map_value").value)
        self.cell_stride = max(1, int(self.get_parameter("cell_stride").value))
        self.z_offset = float(self.get_parameter("z_offset").value)
        self.robot_a_alpha = float(self.get_parameter("robot_a_alpha").value)
        self.robot_b_alpha = float(self.get_parameter("robot_b_alpha").value)
        self.robot_marker_size = float(self.get_parameter("robot_marker_size").value)
        self.robot_marker_alpha = float(self.get_parameter("robot_marker_alpha").value)
        self.robot_label_scale = float(self.get_parameter("robot_label_scale").value)
        self.robot_label_z_offset = float(self.get_parameter("robot_label_z_offset").value)
        self.robot_a_ns = self._extract_namespace(self.robot_a_map_topic, "go2_1")
        self.robot_b_ns = self._extract_namespace(self.robot_b_map_topic, "go2_2")

        self.robot_a_map: Optional[OccupancyGrid] = None
        self.robot_b_map: Optional[OccupancyGrid] = None
        self.robot_a_odom: Optional[Odometry] = None
        self.robot_b_odom: Optional[Odometry] = None

        self.create_subscription(OccupancyGrid, self.robot_a_map_topic, self._robot_a_map_cb, 1)
        self.create_subscription(OccupancyGrid, self.robot_b_map_topic, self._robot_b_map_cb, 1)
        self.create_subscription(Odometry, self.robot_a_odom_topic, self._robot_a_odom_cb, 10)
        self.create_subscription(Odometry, self.robot_b_odom_topic, self._robot_b_odom_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate), self._publish_markers)

        self.get_logger().info(
            f"Dual coverage visualizer started: {self.robot_a_map_topic}, {self.robot_b_map_topic} -> {self.marker_topic}"
        )

    def _robot_a_map_cb(self, msg: OccupancyGrid) -> None:
        self.robot_a_map = msg

    def _robot_b_map_cb(self, msg: OccupancyGrid) -> None:
        self.robot_b_map = msg

    def _robot_a_odom_cb(self, msg: Odometry) -> None:
        self.robot_a_odom = msg

    def _robot_b_odom_cb(self, msg: Odometry) -> None:
        self.robot_b_odom = msg

    def _publish_markers(self) -> None:
        markers = MarkerArray()

        if self.robot_a_map is not None:
            markers.markers.append(
                self._build_coverage_marker(
                    map_msg=self.robot_a_map,
                    frame_id=self._resolve_frame(self.robot_a_map),
                    marker_id=0,
                    marker_ns=f"{self.robot_a_ns}_coverage",
                    r=1.0,
                    g=0.0,
                    b=0.0,
                    a=self.robot_a_alpha,
                )
            )

        if self.robot_b_map is not None:
            markers.markers.append(
                self._build_coverage_marker(
                    map_msg=self.robot_b_map,
                    frame_id=self._resolve_frame(self.robot_b_map),
                    marker_id=1,
                    marker_ns=f"{self.robot_b_ns}_coverage",
                    r=0.0,
                    g=1.0,
                    b=0.0,
                    a=self.robot_b_alpha,
                )
            )

        if self.robot_a_odom is not None:
            frame_id = self._resolve_robot_frame(self.robot_a_map, self.robot_a_odom)
            markers.markers.append(
                self._build_robot_marker(
                    odom_msg=self.robot_a_odom,
                    frame_id=frame_id,
                    marker_id=100,
                    marker_ns=f"{self.robot_a_ns}_pose",
                    r=1.0,
                    g=0.1,
                    b=0.1,
                )
            )
            markers.markers.append(
                self._build_robot_label_marker(
                    odom_msg=self.robot_a_odom,
                    frame_id=frame_id,
                    marker_id=200,
                    marker_ns=f"{self.robot_a_ns}_label",
                    text=self.robot_a_ns,
                    r=1.0,
                    g=0.2,
                    b=0.2,
                )
            )

        if self.robot_b_odom is not None:
            frame_id = self._resolve_robot_frame(self.robot_b_map, self.robot_b_odom)
            markers.markers.append(
                self._build_robot_marker(
                    odom_msg=self.robot_b_odom,
                    frame_id=frame_id,
                    marker_id=101,
                    marker_ns=f"{self.robot_b_ns}_pose",
                    r=0.1,
                    g=1.0,
                    b=0.1,
                )
            )
            markers.markers.append(
                self._build_robot_label_marker(
                    odom_msg=self.robot_b_odom,
                    frame_id=frame_id,
                    marker_id=201,
                    marker_ns=f"{self.robot_b_ns}_label",
                    text=self.robot_b_ns,
                    r=0.2,
                    g=1.0,
                    b=0.2,
                )
            )

        if markers.markers:
            self.marker_pub.publish(markers)

    def _build_coverage_marker(
        self,
        map_msg: OccupancyGrid,
        frame_id: str,
        marker_id: int,
        marker_ns: str,
        r: float,
        g: float,
        b: float,
        a: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = map_msg.info.resolution
        marker.scale.y = map_msg.info.resolution
        marker.scale.z = 0.02
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = max(0.0, min(1.0, a))

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        oz = map_msg.info.origin.position.z + self.z_offset

        data = map_msg.data
        stride = self.cell_stride

        for gy in range(0, height, stride):
            row_offset = gy * width
            for gx in range(0, width, stride):
                value = data[row_offset + gx]
                if value < self.min_map_value:
                    continue

                pt = Point()
                pt.x = ox + (gx + 0.5) * resolution
                pt.y = oy + (gy + 0.5) * resolution
                pt.z = oz
                marker.points.append(pt)

        return marker

    def _build_robot_marker(
        self,
        odom_msg: Odometry,
        frame_id: str,
        marker_id: int,
        marker_ns: str,
        r: float,
        g: float,
        b: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = odom_msg.pose.pose
        marker.scale.x = self.robot_marker_size
        marker.scale.y = self.robot_marker_size
        marker.scale.z = self.robot_marker_size
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = max(0.0, min(1.0, self.robot_marker_alpha))
        return marker

    def _build_robot_label_marker(
        self,
        odom_msg: Odometry,
        frame_id: str,
        marker_id: int,
        marker_ns: str,
        text: str,
        r: float,
        g: float,
        b: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = odom_msg.pose.pose.position
        marker.pose.position.z += self.robot_label_z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.robot_label_scale
        marker.text = text
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = max(0.0, min(1.0, self.robot_marker_alpha))
        return marker

    def _resolve_frame(self, map_msg: Optional[OccupancyGrid]) -> str:
        if self.marker_frame:
            return self.marker_frame
        if map_msg is not None and map_msg.header.frame_id:
            return map_msg.header.frame_id
        return "world"

    @staticmethod
    def _extract_namespace(topic: str, fallback: str) -> str:
        clean = topic.strip("/")
        if not clean:
            return fallback
        return clean.split("/")[0]

    def _resolve_robot_frame(self, map_msg: Optional[OccupancyGrid], odom_msg: Odometry) -> str:
        if self.marker_frame:
            return self.marker_frame
        if map_msg is not None and map_msg.header.frame_id:
            return map_msg.header.frame_id
        if odom_msg.header.frame_id:
            return odom_msg.header.frame_id
        return "world"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualMapCoverageVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
