#!/usr/bin/env python3
"""terrain_height_filter — single-param obstacle-height threshold.

Consumes terrain_analysis output (PointXYZI in `map` frame, where
`intensity = point.z − local_ground_z` per voxel), drops every point
with `intensity < min_obstacle_height_m`, republishes the remainder
as a plain XYZ PointCloud2.

Why this exists: the legacy stack had 7 different height filters
across 4 reference frames (base_link / map / vehicle / local-ground),
all needing tuning together and disagreeing on ramps. This node
centralises height filtering on the one quantity that is robot-pose-
and ramp-independent — "obstacle height above the ground beneath it" —
and exposes a single hyperparameter `min_obstacle_height_m` that
downstream consumers (octomap, localPlanner, far_planner) all bind
to.

Input  : sensor_msgs/PointCloud2  (PointXYZI, frame=map)
         topics: `input_topic` + optional `extra_input_topics`
         (terrain_analysis publishes /{ns}/terrain_map; the extended-
          range variant publishes /{ns}/terrain_map_ext.)

Output : sensor_msgs/PointCloud2  (PointXYZ, frame=map)
         on `output_topic` — only points with intensity ≥ threshold.

The node is stateless, does no TF lookup, runs sub-millisecond per
scan; it merges multiple inputs by simply forwarding each filtered
result independently (downstream octomap accumulates them anyway).
"""

from __future__ import annotations

import struct
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class TerrainHeightFilter(Node):
    def __init__(self) -> None:
        super().__init__("terrain_height_filter")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("min_obstacle_height_m", 0.05)
        self.declare_parameter("input_topic", "terrain_map")
        self.declare_parameter("output_topic", "terrain_obstacle_cloud")
        self.declare_parameter(
            "extra_input_topics", ["terrain_map_ext"]
        )
        self.declare_parameter("status_topic", "terrain_height_filter_status")
        self.declare_parameter("status_period_sec", 10.0)

        self.min_h = float(self.get_parameter("min_obstacle_height_m").value)
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        extra_topics = list(self.get_parameter("extra_input_topics").value or [])
        status_topic = str(self.get_parameter("status_topic").value)
        status_period = float(self.get_parameter("status_period_sec").value)

        # ── State ─────────────────────────────────────────────────────
        self._scans_in = 0
        self._points_in = 0
        self._points_out = 0

        # ── Pub / sub ─────────────────────────────────────────────────
        # terrain_map publishers from CMU run BEST_EFFORT / VOLATILE; match.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Octomap consumes RELIABLE; publish that way (octomap_server's
        # cloud_in will still match — RELIABLE pub + BEST_EFFORT sub
        # is a compatible pairing).
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(PointCloud2, out_topic, out_qos)
        self.status_pub = self.create_publisher(String, status_topic, 5)

        all_inputs = [in_topic] + [t for t in extra_topics if t and t.strip()]
        for t in all_inputs:
            self.create_subscription(PointCloud2, t, self._on_cloud, sensor_qos)

        if status_period > 0.0:
            self.create_timer(status_period, self._on_status_tick)

        self.get_logger().info(
            f"terrain_height_filter armed. min_obstacle_height={self.min_h:.3f} m "
            f"| inputs={all_inputs} → output={out_topic}"
        )

    # ── Callbacks ──────────────────────────────────────────────────────
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._scans_in += 1

        # Locate x, y, z, intensity offsets in the input cloud.
        offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
        if not all(k in offsets for k in ("x", "y", "z", "intensity")):
            # Not a PointXYZI cloud — pass through unchanged so downstream
            # doesn't starve. terrain_analysis always publishes XYZI but
            # belt-and-braces for forks / mocked inputs.
            self.pub.publish(msg)
            return

        x_off, x_dt = offsets["x"]
        y_off, y_dt = offsets["y"]
        z_off, z_dt = offsets["z"]
        i_off, i_dt = offsets["intensity"]

        # All four must be FLOAT32 — that's what terrain_analysis emits.
        FLOAT32 = PointField.FLOAT32
        if not (x_dt == FLOAT32 and y_dt == FLOAT32 and z_dt == FLOAT32 and i_dt == FLOAT32):
            self.pub.publish(msg)
            return

        N = msg.width * msg.height
        step = msg.point_step
        if N == 0 or step == 0 or msg.data is None or len(msg.data) != N * step:
            return

        threshold = self.min_h
        src = msg.data

        # One pass: keep points whose intensity ≥ threshold. Pack into a
        # tight XYZ-only float32 buffer (12 bytes/point) for the output
        # cloud — octomap and other consumers don't need intensity.
        out_step = 12  # 3 × float32
        out_buf = bytearray(N * out_step)
        kept = 0
        for i in range(N):
            base = i * step
            (intensity,) = struct.unpack_from("<f", src, base + i_off)
            if intensity < threshold:
                continue
            # Copy x, y, z verbatim
            x_b = src[base + x_off : base + x_off + 4]
            y_b = src[base + y_off : base + y_off + 4]
            z_b = src[base + z_off : base + z_off + 4]
            o = kept * out_step
            out_buf[o : o + 4] = x_b
            out_buf[o + 4 : o + 8] = y_b
            out_buf[o + 8 : o + 12] = z_b
            kept += 1

        self._points_in += N
        self._points_out += kept

        out = PointCloud2()
        out.header = msg.header  # frame=map, stamp preserved
        out.height = 1
        out.width = kept
        out.is_bigendian = False
        out.is_dense = True
        out.point_step = out_step
        out.row_step = out_step * kept
        out.fields = [
            PointField(name="x", offset=0, datatype=FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=FLOAT32, count=1),
        ]
        out.data = bytes(out_buf[: kept * out_step])
        self.pub.publish(out)

    def _on_status_tick(self) -> None:
        if self._scans_in == 0:
            keep_pct = 0.0
        else:
            keep_pct = (
                100.0 * self._points_out / max(1, self._points_in)
            )
        self.get_logger().info(
            f"terrain_height_filter: {self._scans_in} scans, "
            f"{self._points_in} pts in, {self._points_out} pts out "
            f"({keep_pct:.1f}% kept @ ≥{self.min_h:.3f}m)"
        )
        s = String()
        s.data = (
            '{"schema":"terrain_height_filter/v1"'
            f',"scans":{self._scans_in}'
            f',"points_in":{self._points_in}'
            f',"points_out":{self._points_out}'
            f',"min_obstacle_height_m":{self.min_h:.3f}'
            "}"
        )
        self.status_pub.publish(s)


def main(argv=None) -> int:
    ap_args, ros_argv = _split_argv(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = TerrainHeightFilter()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    return 0


def _split_argv(argv):
    """Separate ROS-args (after --ros-args) from anything else.
    The node has no app-level args; everything we accept goes via
    ros2 parameters."""
    if "--ros-args" in argv:
        idx = argv.index("--ros-args")
        return argv[:idx], argv[idx:]
    return argv, []


if __name__ == "__main__":
    sys.exit(main())
