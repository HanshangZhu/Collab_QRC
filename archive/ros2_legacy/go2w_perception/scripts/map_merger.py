#!/usr/bin/env python3
"""Merge two OccupancyGrid topics via a cell-wise union.

Used to give astar_nav_node obstacle evidence from BOTH Cartographer's
binarized map (stable walls, slow to update) AND octomap_server's
projected_map (fast thin-obstacle detection). Because the planner
subscribes to exactly one map topic, we compute the union upstream and
publish it with the QoS that the planner expects.

Coordinate handling:
    Cartographer and octomap both use the same `map` frame but may have
    slightly different grid origins. We treat the primary grid as the
    canonical coordinate reference, then for every occupied cell in the
    secondary grid we compute its world (x, y) and map it back into the
    primary's cell index, setting the merged cell occupied.

Graceful degradation:
    - Before either topic has been seen, publish nothing.
    - Once primary has been seen, publish primary-as-is (so the planner
      always has a map the moment Cartographer comes up, even if octomap
      is still warming up).
    - Once both have been seen, publish the union on every primary update.
"""
from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _primary_qos() -> QoSProfile:
    """QoS matching Cartographer binarizer's OccupancyGrid publisher."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _secondary_qos() -> QoSProfile:
    """QoS matching octomap_server's projected_map publisher (volatile)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _output_qos() -> QoSProfile:
    """QoS matching what astar_nav_node expects (TRANSIENT_LOCAL for map)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MapMerger(Node):
    def __init__(self) -> None:
        super().__init__("map_merger")

        self.declare_parameter("primary_topic", "/robot/map")
        self.declare_parameter("secondary_topic", "/robot/octomap_projected_map")
        self.declare_parameter("output_topic", "/robot/map_merged")
        self.declare_parameter("secondary_occupied_thresh", 50)
        self.declare_parameter("publish_rate_hz", 4.0)
        # Radius (in cells) to dilate secondary occupied cells before union.
        # Gives thin-obstacle detections a small safety buffer so that a
        # single-cell-wide divider wall grows to a 3-cell stripe.
        self.declare_parameter("secondary_dilate_cells", 1)

        self.primary_topic = str(self.get_parameter("primary_topic").value)
        self.secondary_topic = str(self.get_parameter("secondary_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.sec_thresh = int(self.get_parameter("secondary_occupied_thresh").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.sec_dilate = int(self.get_parameter("secondary_dilate_cells").value)

        self._last_primary: OccupancyGrid | None = None
        self._last_secondary: OccupancyGrid | None = None
        self._last_merged_cells_added = 0
        self._last_log_sec = 0.0

        self.create_subscription(
            OccupancyGrid, self.primary_topic, self._on_primary, _primary_qos()
        )
        self.create_subscription(
            OccupancyGrid, self.secondary_topic, self._on_secondary, _secondary_qos()
        )

        self._pub = self.create_publisher(
            OccupancyGrid, self.output_topic, _output_qos()
        )

        period = max(0.05, 1.0 / max(0.1, self.publish_rate_hz))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"map_merger: primary='{self.primary_topic}' "
            f"secondary='{self.secondary_topic}' -> output='{self.output_topic}' "
            f"(sec_thresh={self.sec_thresh}, fallback_rate={self.publish_rate_hz} Hz)"
        )

    # ── Callbacks ─────────────────────────────────────────────────────
    def _on_primary(self, msg: OccupancyGrid) -> None:
        self._last_primary = msg
        self._publish_merge()

    def _on_secondary(self, msg: OccupancyGrid) -> None:
        self._last_secondary = msg

    def _tick(self) -> None:
        # Fallback timer — keeps the output alive when primary is silent.
        if self._last_primary is None:
            return
        self._publish_merge()

    # ── Merge core ────────────────────────────────────────────────────
    def _publish_merge(self) -> None:
        primary = self._last_primary
        if primary is None:
            return
        merged = self._merge(primary, self._last_secondary)
        merged.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(merged)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_log_sec >= 5.0:
            self._last_log_sec = now_sec
            sec_state = "none" if self._last_secondary is None else "ok"
            self.get_logger().info(
                f"map_merger: primary_cells={primary.info.width * primary.info.height} "
                f"secondary={sec_state} sec_added={self._last_merged_cells_added}"
            )

    def _merge(
        self, primary: OccupancyGrid, secondary: OccupancyGrid | None
    ) -> OccupancyGrid:
        out = OccupancyGrid()
        out.header = primary.header
        out.info = primary.info
        # Copy primary data as the baseline.
        base = np.asarray(primary.data, dtype=np.int8)
        self._last_merged_cells_added = 0

        if secondary is None or secondary.info.width == 0 or secondary.info.height == 0:
            out.data = base.tolist()
            return out

        sec_arr = np.asarray(secondary.data, dtype=np.int16).reshape(
            secondary.info.height, secondary.info.width
        )
        occ_mask = sec_arr >= self.sec_thresh
        if not np.any(occ_mask):
            out.data = base.tolist()
            return out

        # Dilate the secondary occupancy mask by a small radius so thin
        # obstacles get a safety buffer before the merge. Uses a square
        # structuring element via numpy roll + OR — cheap, no scipy dep.
        if self.sec_dilate > 0:
            r = self.sec_dilate
            dilated = occ_mask.copy()
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    if dj == 0 and di == 0:
                        continue
                    dilated |= np.roll(np.roll(occ_mask, dj, axis=0), di, axis=1)
            occ_mask = dilated

        # World coords of each occupied secondary cell.
        sec_res = secondary.info.resolution
        sec_ox = secondary.info.origin.position.x
        sec_oy = secondary.info.origin.position.y
        j_idx, i_idx = np.nonzero(occ_mask)
        world_x = sec_ox + (i_idx.astype(np.float64) + 0.5) * sec_res
        world_y = sec_oy + (j_idx.astype(np.float64) + 0.5) * sec_res

        # Map into primary's cell indices.
        pri_res = primary.info.resolution
        pri_ox = primary.info.origin.position.x
        pri_oy = primary.info.origin.position.y
        pri_w = primary.info.width
        pri_h = primary.info.height

        px = np.floor((world_x - pri_ox) / pri_res).astype(np.int64)
        py = np.floor((world_y - pri_oy) / pri_res).astype(np.int64)
        in_bounds = (px >= 0) & (px < pri_w) & (py >= 0) & (py < pri_h)
        px = px[in_bounds]
        py = py[in_bounds]

        merged = base.reshape(pri_h, pri_w).copy()
        # Count additions before we overwrite so the log shows true new occupancy.
        new_hits = np.count_nonzero(merged[py, px] != 100)
        merged[py, px] = 100
        self._last_merged_cells_added = int(new_hits)

        out.data = merged.flatten().tolist()
        return out


def main() -> None:
    rclpy.init()
    node = MapMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
