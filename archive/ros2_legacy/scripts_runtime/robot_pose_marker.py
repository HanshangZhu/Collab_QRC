#!/usr/bin/env python3
"""robot_pose_marker — synthesize the RViz red triangle from /<ns>/odom/nav.

Nav2 MPPI's controller_server/planner_server don't publish the robot pose
marker that the legacy A*/default_nav backends do, so on nav=nav2_mppi the
RViz `RobotPoseTriangle` display has no publisher and stays blank. This
node fills that gap: subscribes /<ns>/odom/nav and emits the same
visualization_msgs/Marker (TRIANGLE_LIST, red, ~0.7×0.35 m Go2W footprint)
that default_nav.py produces — so the same RViz config works across all
nav backends.

Usage:
    python3 robot_pose_marker.py --ros-args \
        -p namespace:=robot \
        -p frame_id:=map \
        -p length:=0.70 -p width:=0.35
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class RobotPoseMarker(Node):
    def __init__(self) -> None:
        super().__init__("robot_pose_marker")
        self.declare_parameter("namespace", "robot")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("length", 0.70)
        self.declare_parameter("width", 0.35)
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("marker_topic", "robot_pose_marker")

        ns = str(self.get_parameter("namespace").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._half_length = float(self.get_parameter("length").value) / 2.0
        self._half_width = float(self.get_parameter("width").value) / 2.0

        odom_topic = f"/{ns}/{self.get_parameter('odom_topic').value}"
        marker_topic = f"/{ns}/{self.get_parameter('marker_topic').value}"

        # RELIABLE/VOLATILE matches RViz's Path/Marker default subscription QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Marker, marker_topic, qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)

        self.get_logger().info(
            f"robot_pose_marker up: ns={ns} {odom_topic} → {marker_topic} "
            f"(frame={self._frame_id}, "
            f"size={2 * self._half_length:.2f}×{2 * self._half_width:.2f}m)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = self._frame_id
        marker.ns = "robot_pose"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.10
        marker.color.b = 0.10
        marker.color.a = 0.90

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        # Front tip + rear-left + rear-right (TRIANGLE_LIST = single triangle).
        p0 = Point(); p0.x = cx + self._half_length * cos_y;            p0.y = cy + self._half_length * sin_y;            p0.z = 0.05
        p1 = Point(); p1.x = cx - self._half_length * cos_y - self._half_width * sin_y; p1.y = cy - self._half_length * sin_y + self._half_width * cos_y; p1.z = 0.05
        p2 = Point(); p2.x = cx - self._half_length * cos_y + self._half_width * sin_y; p2.y = cy - self._half_length * sin_y - self._half_width * cos_y; p2.z = 0.05
        marker.points = [p0, p1, p2]
        self._pub.publish(marker)


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        rclpy.spin(RobotPoseMarker())
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
