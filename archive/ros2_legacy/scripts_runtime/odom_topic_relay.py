#!/usr/bin/env python3
"""odom_topic_relay — minimal Odometry relay.

Used onboard the Jetson to bridge Fast-LIO's un-namespaced /Odometry
to /<namespace>/odom/nav, which is what the laptop's Nav2 stack
subscribes to. Equivalent to:
    ros2 run topic_tools relay /Odometry /<ns>/odom/nav

Reason this exists separately from fast_lio_tf_adapter.py: the full
adapter segfaults inside Foxy's rclpy 0.9.x (works fine on Humble).
This stripped-down node is small enough to dodge whichever
incompatibility was triggering the segfault — and ros-foxy-topic-tools
isn't always installed on the Jetson.

Usage:
    python3 odom_topic_relay.py --ros-args \
        -p input_topic:=/Odometry \
        -p output_topic:=/robot/odom/nav
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_topic_relay")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/robot/odom/nav")
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._on_msg, 10)
        self.get_logger().info(f"odom_topic_relay: {in_topic} -> {out_topic}")

    def _on_msg(self, msg):
        self._pub.publish(msg)


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        rclpy.spin(OdomRelay())
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
