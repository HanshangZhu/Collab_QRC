#!/usr/bin/env python3
"""Tiny path relay so RViz picks up Nav2's plans on the topic names
the existing nav_test_mixed.rviz config expects.

Nav2 publishes:
  /<ns>/plan          (global SmacHybrid path)
  /<ns>/local_plan    (controller-local path; MPPI publishes optimal)

Existing RViz config subscribes:
  /<ns>/planned_path
  /<ns>/local_path

Run alongside the Nav2 stack to bridge the topic names.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from nav_msgs.msg import Path


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class PathRelay(Node):
    def __init__(self) -> None:
        super().__init__("path_relay")
        self.declare_parameter("namespace", "robot_a")
        ns = str(self.get_parameter("namespace").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._plan_pub = self.create_publisher(Path, f"/{ns}/planned_path", qos)
        self._local_pub = self.create_publisher(Path, f"/{ns}/local_path", qos)

        self.create_subscription(
            Path, f"/{ns}/plan",
            lambda m: self._plan_pub.publish(m), qos)
        self.create_subscription(
            Path, f"/{ns}/local_plan",
            lambda m: self._local_pub.publish(m), qos)

        self.get_logger().info(
            f"path_relay: /{ns}/plan→/{ns}/planned_path, "
            f"/{ns}/local_plan→/{ns}/local_path"
        )


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        rclpy.spin(PathRelay())
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
