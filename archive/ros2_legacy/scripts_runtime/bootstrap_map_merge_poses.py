#!/usr/bin/env python3
"""One-shot helper: bootstrap multirobot_map_merge's per-robot init_pose
params from each robot's first published /map message.

The `multirobot_map_merge` node has two quirks we work around here:

1. **`init_pose_{x,y}` is in pixels, not meters.** Despite the docs, the
   C++ uses `translation.{x,y}` directly in a `cv::Mat` that's passed
   straight to `cv::warpAffine`, which interprets the translation columns
   as integer pixel offsets. See pipeline.cpp:126-127.

2. **Input grids' `info.origin` is ignored.** The merger composes grids
   using their cell indices only; where each grid sits in the world
   comes entirely from `init_pose`. See pipeline.cpp:202-207.

So for a shared-frame setup (both robots publish /map in frame_id=map,
static TF world→map is identity), we capture each robot's OccupancyGrid
`info.origin` at startup and convert to pixel offsets:

  init_pose_x = info.origin.x / info.resolution   # pixels
  init_pose_y = info.origin.y / info.resolution
  init_pose_yaw = 0   # octomap never rotates the grid

This tells the merger where to place each robot's cell (0,0) on the
merged canvas, which is what the merger actually needs.

Also captures the ground-truth pose from `/robot/odom/ground_truth` for
logging / sanity-checking (not used for the init_pose values).

Runs once, writes YAML, exits 0. An OnProcessExit handler in the launch
file chains the map_merge node onto that exit.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class BootstrapPoses(Node):

    def __init__(self, robots: list[str], gt_topic_suffix: str,
                 map_topic_suffix: str) -> None:
        super().__init__("bootstrap_map_merge_poses")
        self._robots = robots
        self._gt: Dict[str, Optional[Odometry]] = {r: None for r in robots}
        self._map: Dict[str, Optional[OccupancyGrid]] = {r: None for r in robots}
        self._subs = []

        # Maps are published transient_local/reliable by octomap_server.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        gt_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        for r in robots:
            gt_topic = f"/{r}/{gt_topic_suffix.lstrip('/')}"
            map_topic = f"/{r}/{map_topic_suffix.lstrip('/')}"
            self._subs.append(self.create_subscription(
                Odometry, gt_topic,
                lambda msg, rname=r: self._on_gt(rname, msg), gt_qos))
            self._subs.append(self.create_subscription(
                OccupancyGrid, map_topic,
                lambda msg, rname=r: self._on_map(rname, msg), map_qos))
            self.get_logger().info(
                f"waiting for {gt_topic} and {map_topic}"
            )

    def _on_gt(self, robot: str, msg: Odometry) -> None:
        if self._gt[robot] is None:
            self._gt[robot] = msg
            p = msg.pose.pose
            yaw = _yaw_from_quat(p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w)
            self.get_logger().info(
                f"GT {robot}: x={p.position.x:.3f} y={p.position.y:.3f} "
                f"yaw={math.degrees(yaw):.2f}°"
            )

    def _on_map(self, robot: str, msg: OccupancyGrid) -> None:
        if self._map[robot] is None:
            self._map[robot] = msg
            o = msg.info.origin.position
            self.get_logger().info(
                f"MAP {robot}: origin=({o.x:.3f}, {o.y:.3f}) "
                f"res={msg.info.resolution:.3f} "
                f"size={msg.info.width}x{msg.info.height}"
            )

    def all_captured(self) -> bool:
        return all(g is not None for g in self._gt.values()) and \
               all(m is not None for m in self._map.values())

    def init_poses_pixels(self) -> Dict[str, Tuple[float, float, float]]:
        """Per-robot (init_pose_x_px, init_pose_y_px, yaw_rad)."""
        out: Dict[str, Tuple[float, float, float]] = {}
        for r, m in self._map.items():
            assert m is not None
            res = m.info.resolution
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            # map_merge treats init_pose as pixels; convert meters → pixels.
            out[r] = (ox / res, oy / res, 0.0)
        return out

    def gt_summary(self) -> Dict[str, Tuple[float, float, float]]:
        out: Dict[str, Tuple[float, float, float]] = {}
        for r, msg in self._gt.items():
            assert msg is not None
            p = msg.pose.pose
            yaw = _yaw_from_quat(p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w)
            out[r] = (p.position.x, p.position.y, yaw)
        return out


def _write_yaml(path: str,
                init_poses_px: Dict[str, Tuple[float, float, float]],
                gt_meters: Dict[str, Tuple[float, float, float]],
                merged_map_topic: str, merging_rate: float,
                discovery_rate: float) -> None:
    lines: list[str] = []
    lines.append("# Auto-generated by bootstrap_map_merge_poses.py — do not edit.")
    lines.append("# multirobot_map_merge treats `init_pose_{x,y}` as PIXELS (the")
    lines.append("# C++ applies translation directly in cv::warpAffine), so we")
    lines.append("# convert each robot's map.info.origin from meters → pixels")
    lines.append("# using its own map resolution.")
    lines.append("#")
    lines.append("# Reference: GT spawn poses captured at the same moment:")
    for robot, (x, y, yaw) in gt_meters.items():
        lines.append(f"#   {robot} GT: x={x:.3f}m y={y:.3f}m yaw={math.degrees(yaw):.2f}°")
    lines.append("")
    lines.append("map_merge:")
    lines.append("  ros__parameters:")
    lines.append("    known_init_poses: true")
    lines.append(f"    merging_rate: {merging_rate}")
    lines.append(f"    discovery_rate: {discovery_rate}")
    lines.append("    estimation_rate: 0.5")
    lines.append("    estimation_confidence: 0.6")
    lines.append("    robot_map_topic: map")
    lines.append("    robot_map_updates_topic: map_updates")
    lines.append('    robot_namespace: ""')
    lines.append(f"    merged_map_topic: {merged_map_topic}")
    lines.append("    world_frame: world")
    lines.append("")
    for robot, (x_px, y_px, yaw) in init_poses_px.items():
        lines.append(f"    /{robot}/map_merge/init_pose_x: {x_px:.6f}")
        lines.append(f"    /{robot}/map_merge/init_pose_y: {y_px:.6f}")
        lines.append(f"    /{robot}/map_merge/init_pose_z: 0.0")
        lines.append(f"    /{robot}/map_merge/init_pose_yaw: {yaw:.6f}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+", required=True)
    ap.add_argument("--gt-topic-suffix", default="odom/ground_truth")
    ap.add_argument("--map-topic-suffix", default="map")
    ap.add_argument("--output", required=True)
    ap.add_argument("--timeout-sec", type=float, default=60.0)
    ap.add_argument("--merged-map-topic", default="merged_map")
    ap.add_argument("--merging-rate", type=float, default=2.0)
    ap.add_argument("--discovery-rate", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = BootstrapPoses(args.robots, args.gt_topic_suffix,
                          args.map_topic_suffix)

    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and not node.all_captured():
            if time.monotonic() >= deadline:
                missing_gt = [r for r, v in node._gt.items() if v is None]
                missing_map = [r for r, v in node._map.items() if v is None]
                node.get_logger().error(
                    f"timed out after {args.timeout_sec}s; "
                    f"missing GT from {missing_gt}, missing MAP from {missing_map}"
                )
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 130
    finally:
        init_px = node.init_poses_pixels() if node.all_captured() else None
        gt_m = node.gt_summary() if all(v is not None for v in node._gt.values()) else None
        node.destroy_node()
        rclpy.try_shutdown()

    if init_px is None or gt_m is None:
        return 2

    _write_yaml(
        args.output, init_px, gt_m,
        merged_map_topic=args.merged_map_topic,
        merging_rate=args.merging_rate,
        discovery_rate=args.discovery_rate,
    )
    print(f"[bootstrap_map_merge_poses] wrote {args.output}")
    for robot, (x_px, y_px, _) in init_px.items():
        print(f"  {robot}: init_pose=({x_px:.2f}, {y_px:.2f}) px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
