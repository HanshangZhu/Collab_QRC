#!/usr/bin/env python3
"""Temporal median filter for LaserScan messages.

Maintains a sliding window of the last N scans and publishes a filtered
scan where each angle bin uses the **median** range over the window.
Transient noise (single-frame self-observations, ground bounces, scan
smear during fast motion) gets rejected because it doesn't survive the
median across multiple frames.

This is conceptually similar to a particle filter: only observations
that appear consistently across time are kept.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class TemporalScanFilter(Node):
    def __init__(self):
        super().__init__("temporal_scan_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("window_size", 5)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.window_size = max(2, self.get_parameter("window_size").value)

        self.buffer: deque[np.ndarray] = deque(maxlen=self.window_size)

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(LaserScan, input_topic, self.on_scan, qos_in)
        self.pub = self.create_publisher(LaserScan, output_topic, qos_out)

        self.msg_count = 0
        self.get_logger().info(
            f"Temporal scan filter: {input_topic} -> {output_topic} "
            f"(window={self.window_size})"
        )

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Replace inf/nan with max_range so median math works
        bad = ~np.isfinite(ranges)
        ranges[bad] = msg.range_max

        self.buffer.append(ranges)

        if len(self.buffer) < 2:
            # Not enough history — pass through unfiltered
            self.pub.publish(msg)
            return

        # Stack all buffered scans and take per-angle median
        stacked = np.stack(self.buffer, axis=0)  # shape: (window, num_rays)
        median_ranges = np.median(stacked, axis=0).astype(np.float32)

        # Build output message preserving all metadata from latest scan
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = median_ranges.tolist()
        out.intensities = msg.intensities  # pass through unchanged

        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 200 == 1:
            valid = median_ranges[median_ranges < msg.range_max * 0.99]
            self.get_logger().info(
                f"Filtered {self.msg_count} scans | "
                f"buffer={len(self.buffer)}/{self.window_size} | "
                f"valid_rays={len(valid)}/{len(median_ranges)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TemporalScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
