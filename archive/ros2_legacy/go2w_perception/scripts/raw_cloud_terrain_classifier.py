#!/usr/bin/env python3
"""raw_cloud_terrain_classifier — semantic alignment of height filtering.

Replaces the world-z slice that octomap was using with the same per-voxel
local-ground reference that terrain_analysis / FAR / localPlanner use.

Pipeline:
   raw cloud (map frame) ──┐
                            │ terrain_classifier
   /terrain_map (XYZI) ─────┘    │
                                 │  classified cloud:
                                 │    obstacle pts at real z
                                 │    ground   pts at z = threshold − ε
                                 ▼
                            octomap_server (occupancy_min_z = threshold)
                                 │ rays from sensor preserve free-space carving
                                 │ ground voxels excluded from /map projection
                                 ▼
                              /map  (consistent semantics with FAR / localPlanner;
                                     ramp-correct because intensity is per-voxel
                                     local-ground)

The single parameter `min_obstacle_height_m` now means the same geometric
quantity everywhere it appears in the stack:

  octomap occupancy_min_z         → threshold for projection of CLASSIFIED cloud
  terrain_height_filter / FAR /
   localPlanner intensity threshold → per-voxel local-ground reference

On flat ground the world-z slice and per-voxel intensity coincide; on
ramps they diverge, and we now use only the per-voxel form.

Implementation:
  1. terrain_map callback rebuilds a {(x_bin, y_bin) → local_ground_z}
     hashmap at 0.2 m resolution. local_ground_z is reconstructed as
     `point.z − point.intensity` (terrain_analysis's definition).
  2. raw cloud callback iterates every point:
       bin = (int(rx*5), int(ry*5))    # 0.2 m bin
       g_z = bin_map.get(bin, default_floor)
       if rz − g_z < min_obstacle_height_m:
           rewrite output z = min_obstacle_height_m − 0.01
       else:
           output z = rz
     publishes the rewritten cloud unchanged in xy.
  3. octomap consumes the classified cloud directly. Its
     `occupancy_min_z` filters projection of ground voxels.

Performance: 24 k pts/scan, hashmap lookup O(1) per point → sub-ms.
"""

from __future__ import annotations

import math
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


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class RawCloudTerrainClassifier(Node):
    def __init__(self) -> None:
        super().__init__("raw_cloud_terrain_classifier")

        self.declare_parameter("min_obstacle_height_m", 0.05)
        self.declare_parameter("terrain_voxel_size_m", 0.20)
        # When a raw point lands in a bin terrain_analysis hasn't seen,
        # we conservatively assume local_ground_z = `default_ground_z`
        # so the point only registers as obstacle if it's well above the
        # global noise floor. 0.0 keeps existing flat-ground behaviour.
        self.declare_parameter("default_ground_z", 0.0)
        self.declare_parameter("input_cloud_topic", "registered_scan_map")
        self.declare_parameter("input_terrain_topic", "terrain_map")
        self.declare_parameter("output_cloud_topic", "registered_scan_classified")
        self.declare_parameter("status_topic", "raw_cloud_terrain_classifier_status")
        self.declare_parameter("status_period_sec", 10.0)

        self.threshold = float(self.get_parameter("min_obstacle_height_m").value)
        self.bin_size = max(0.05, float(self.get_parameter("terrain_voxel_size_m").value))
        self.default_ground = float(self.get_parameter("default_ground_z").value)
        in_cloud = str(self.get_parameter("input_cloud_topic").value)
        in_terrain = str(self.get_parameter("input_terrain_topic").value)
        out_cloud = str(self.get_parameter("output_cloud_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        status_period = float(self.get_parameter("status_period_sec").value)

        # Inverse bin size for hashing
        self._inv_bin = 1.0 / self.bin_size
        # The z value we rewrite ground points to. Must be < occupancy_min_z
        # (= threshold) so octomap doesn't include them in /map projection,
        # but > -inf so they still receive raycast carving and are inside
        # any reasonable point_cloud_min_z guard.
        self._ground_rewrite_z = self.threshold - 0.01

        # Per-bin local ground table, rebuilt each terrain_map message.
        self._ground_bins: dict[tuple[int, int], float] = {}
        self._ground_bins_stamp = 0.0

        # Stats
        self._scans_in = 0
        self._pts_in = 0
        self._pts_obstacle = 0
        self._pts_ground = 0
        self._pts_default = 0

        # ── QoS ────────────────────────────────────────────────────────
        # registered_scan_map publisher: pointcloud_frame_bridge.py uses
        # default reliable QoS in our launch.
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # terrain_analysis publishes /terrain_map best-effort.
        terrain_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(PointCloud2, in_terrain, self._on_terrain, terrain_qos)
        self.create_subscription(PointCloud2, in_cloud,   self._on_cloud,   cloud_qos)
        self.pub = self.create_publisher(PointCloud2, out_cloud, cloud_qos)
        from std_msgs.msg import String
        self.status_pub = self.create_publisher(String, status_topic, 5)
        self._String = String

        if status_period > 0.0:
            self.create_timer(status_period, self._on_status_tick)

        self.get_logger().info(
            f"raw_cloud_terrain_classifier armed. min_obstacle_height={self.threshold:.3f}m "
            f"bin={self.bin_size:.2f}m | in_cloud={in_cloud} in_terrain={in_terrain} "
            f"out={out_cloud} | ground rewrite z = {self._ground_rewrite_z:.3f}"
        )

    # ── callbacks ──────────────────────────────────────────────────────
    def _on_terrain(self, msg: PointCloud2) -> None:
        """Rebuild the (x_bin, y_bin) → local_ground_z hashmap."""
        offsets = {f.name: f for f in msg.fields}
        if not all(k in offsets for k in ("x", "y", "z", "intensity")):
            return
        if not all(offsets[k].datatype == PointField.FLOAT32
                   for k in ("x", "y", "z", "intensity")):
            return
        x_off = offsets["x"].offset
        y_off = offsets["y"].offset
        z_off = offsets["z"].offset
        i_off = offsets["intensity"].offset
        N = msg.width * msg.height
        step = msg.point_step
        if N == 0 or step == 0 or len(msg.data) != N * step:
            return

        new_bins: dict[tuple[int, int], float] = {}
        inv = self._inv_bin
        data = msg.data
        for i in range(N):
            base = i * step
            x = struct.unpack_from("<f", data, base + x_off)[0]
            y = struct.unpack_from("<f", data, base + y_off)[0]
            z = struct.unpack_from("<f", data, base + z_off)[0]
            inten = struct.unpack_from("<f", data, base + i_off)[0]
            if not (math.isfinite(x) and math.isfinite(y)
                    and math.isfinite(z) and math.isfinite(inten)):
                continue
            ground_z = z - inten
            key = (int(math.floor(x * inv)), int(math.floor(y * inv)))
            # Multiple terrain_map points share a bin; keep the LOWEST
            # ground estimate (terrain_analysis uses 25th-percentile of
            # all points in a bin, but downsampled output may have a
            # few residuals — taking the min is the safest aggregator).
            cur = new_bins.get(key)
            if cur is None or ground_z < cur:
                new_bins[key] = ground_z

        self._ground_bins = new_bins
        self._ground_bins_stamp = self._now_sec()

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Classify each raw point: rewrite ground z, keep obstacle z."""
        offsets = {f.name: f for f in msg.fields}
        if not all(k in offsets for k in ("x", "y", "z")):
            self.pub.publish(msg)
            return
        if not all(offsets[k].datatype == PointField.FLOAT32
                   for k in ("x", "y", "z")):
            self.pub.publish(msg)
            return
        x_off = offsets["x"].offset
        y_off = offsets["y"].offset
        z_off = offsets["z"].offset
        N = msg.width * msg.height
        step = msg.point_step
        if N == 0 or step == 0 or len(msg.data) != N * step:
            return

        # Output: PointXYZ tightly packed (12 bytes/pt) — octomap doesn't
        # need intensity, simpler payload.
        out_step = 12
        out_buf = bytearray(N * out_step)
        bins = self._ground_bins
        inv = self._inv_bin
        threshold = self.threshold
        ground_rewrite_z = self._ground_rewrite_z
        default_ground = self.default_ground
        src = msg.data

        n_obst = 0
        n_ground = 0
        n_default = 0
        for i in range(N):
            base = i * step
            x = struct.unpack_from("<f", src, base + x_off)[0]
            y = struct.unpack_from("<f", src, base + y_off)[0]
            z = struct.unpack_from("<f", src, base + z_off)[0]
            key = (int(math.floor(x * inv)), int(math.floor(y * inv)))
            local_ground = bins.get(key)
            if local_ground is None:
                local_ground = default_ground
                n_default += 1
            if (z - local_ground) < threshold:
                # ground — rewrite z below octomap's projection floor
                out_z = ground_rewrite_z
                n_ground += 1
            else:
                # obstacle — keep real z
                out_z = z
                n_obst += 1
            o = i * out_step
            struct.pack_into("<f", out_buf, o,     x)
            struct.pack_into("<f", out_buf, o + 4, y)
            struct.pack_into("<f", out_buf, o + 8, out_z)

        self._scans_in += 1
        self._pts_in += N
        self._pts_obstacle += n_obst
        self._pts_ground += n_ground
        self._pts_default += n_default

        out = PointCloud2()
        out.header = msg.header  # frame=map, stamp preserved
        out.height = 1
        out.width = N
        out.is_bigendian = False
        out.is_dense = True
        out.point_step = out_step
        out.row_step = out_step * N
        FLOAT32 = PointField.FLOAT32
        out.fields = [
            PointField(name="x", offset=0, datatype=FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=FLOAT32, count=1),
        ]
        out.data = bytes(out_buf)
        self.pub.publish(out)

    def _on_status_tick(self) -> None:
        if self._pts_in == 0:
            return
        obst_pct = 100.0 * self._pts_obstacle / max(1, self._pts_in)
        gnd_pct = 100.0 * self._pts_ground / max(1, self._pts_in)
        def_pct = 100.0 * self._pts_default / max(1, self._pts_in)
        self.get_logger().info(
            f"raw_cloud_terrain_classifier: {self._scans_in} scans, "
            f"{self._pts_in} pts in (obstacle {obst_pct:.1f}%, "
            f"ground {gnd_pct:.1f}%, no-terrain-bin {def_pct:.1f}%) "
            f"@ threshold {self.threshold:.3f}m  bins {len(self._ground_bins)}"
        )
        s = self._String()
        s.data = (
            '{"schema":"raw_cloud_terrain_classifier/v1"'
            f',"scans":{self._scans_in}'
            f',"points":{self._pts_in}'
            f',"obstacle_pct":{obst_pct:.2f}'
            f',"ground_pct":{gnd_pct:.2f}'
            f',"no_terrain_bin_pct":{def_pct:.2f}'
            f',"threshold":{self.threshold:.3f}'
            f',"terrain_bins":{len(self._ground_bins)}'
            "}"
        )
        self.status_pub.publish(s)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = RawCloudTerrainClassifier()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
