#!/usr/bin/env python3
"""Topic-interposition relay that drops a configured fraction of messages.

Subscribes ``in_topic``, drops each message with probability ``drop_prob``
(seeded RNG for reproducibility), republishes survivors to ``out_topic``.
One relay process per dropped directed link. Benchmark scaffolding only —
used to simulate inter-robot coordination-channel loss for the
centralised-vs-decentralised CFPA2 dropout benchmark.
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dropout_core import DropCounter


def _qos(profile: str) -> QoSProfile:
    if profile == "best_effort":
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
    if profile == "transient_local":
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


class CommsDropoutRelay(Node):
    def __init__(self) -> None:
        super().__init__("comms_dropout_relay")
        self.declare_parameter("in_topic", "")
        self.declare_parameter("out_topic", "")
        self.declare_parameter("msg_type", "")  # e.g. nav_msgs/msg/Odometry
        self.declare_parameter("drop_prob", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("qos", "reliable")  # reliable|best_effort|transient_local

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        msg_type = self.get_parameter("msg_type").value
        drop_prob = float(self.get_parameter("drop_prob").value)
        seed = int(self.get_parameter("seed").value)
        qos = _qos(str(self.get_parameter("qos").value))

        if not (in_topic and out_topic and msg_type):
            raise RuntimeError("in_topic, out_topic, msg_type are required")

        self._counter = DropCounter(drop_prob=drop_prob, seed=seed)
        msg_cls = get_message(msg_type)
        self._pub = self.create_publisher(msg_cls, out_topic, qos)
        self._sub = self.create_subscription(msg_cls, in_topic, self._cb, qos)
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"dropout relay {in_topic} -> {out_topic} ({msg_type}) "
            f"p={drop_prob} seed={seed} qos={self.get_parameter('qos').value}")

    def _cb(self, msg) -> None:
        if not self._counter.should_drop():
            self._pub.publish(msg)

    def _log_stats(self) -> None:
        total = self._counter.dropped + self._counter.passed
        frac = (self._counter.dropped / total) if total else 0.0
        self.get_logger().info(
            f"dropped={self._counter.dropped} passed={self._counter.passed} "
            f"frac={frac:.3f}")


def main() -> None:
    rclpy.init()
    node = CommsDropoutRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
