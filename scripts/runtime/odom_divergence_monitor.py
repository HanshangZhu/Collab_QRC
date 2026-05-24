#!/usr/bin/env python3
"""Flags a benchmark trial as invalid when any robot's /odom/nav diverges.

The exploration metrics logger reads GT for robot position (always sane), so
Fast-LIO divergence is invisible there. This monitor subscribes to each
robot's /<ns>/odom/nav, tracks max |x|,|y|, and writes a verdict JSON
(periodically + on shutdown) so the driver can discard + re-run the trial.

Benchmark scaffolding only.
"""
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from divergence_core import is_diverged


class OdomDivergenceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("odom_divergence_monitor")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("odom_topic_suffix", "/odom/nav")
        # dynamic_typing so the bound may be passed as int (60) or float (60.0).
        self.declare_parameter(
            "bound_m", 60.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("out_file", "/tmp/divergence_verdict.json")

        self._ns = list(self.get_parameter("namespaces").value)
        suffix = str(self.get_parameter("odom_topic_suffix").value)
        self._bound = float(self.get_parameter("bound_m").value)
        self._out = str(self.get_parameter("out_file").value)

        self._max_abs = {ns: [0.0, 0.0] for ns in self._ns}
        self._diverged = False
        for ns in self._ns:
            topic = f"/{ns}{suffix}"
            self.create_subscription(
                Odometry, topic, self._make_cb(ns), 10)
            self.get_logger().info(f"watching {topic} (bound={self._bound} m)")
        self.create_timer(2.0, self._write)

    def _make_cb(self, ns):
        def _cb(msg: Odometry) -> None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            m = self._max_abs[ns]
            if x == x:  # not NaN
                m[0] = max(m[0], abs(x))
            if y == y:
                m[1] = max(m[1], abs(y))
            if is_diverged(x, y, self._bound):
                if not self._diverged:
                    self.get_logger().warn(
                        f"DIVERGED: {ns} odom/nav ({x:.1f}, {y:.1f})")
                self._diverged = True
                self._write()
        return _cb

    def _write(self) -> None:
        verdict = {
            "diverged": self._diverged,
            "bound_m": self._bound,
            "max_abs": self._max_abs,
        }
        try:
            with open(self._out, "w") as f:
                json.dump(verdict, f)
        except OSError as exc:
            self.get_logger().error(f"verdict write failed: {exc}")


def main() -> None:
    rclpy.init()
    node = OdomDivergenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
