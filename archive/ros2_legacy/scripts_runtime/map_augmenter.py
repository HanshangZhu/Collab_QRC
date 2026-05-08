#!/usr/bin/env python3
"""map_augmenter — fold swarm-shared knowledge back into THIS robot's
own local OccupancyGrid.

Architecture (matches what real-robot multi-agent deployment looks like):

    [own LiDAR]  ──>  octomap_server  ──>  /{ns}/map_raw   (purely local)
                                                ↓
    [swarm  data]  ──>  /merged_map  ──>  map_augmenter (per ns)
                                                ↓
                                           /{ns}/map      (augmented)
                                                ↓
                                  astar_nav, safety monitor, rviz, …

Cell-merge rule: local cell wins when KNOWN; merged map fills every
other cell. Local sensor values are always trusted over swarm
contributions; swarm contributions populate the gaps AND the area
beyond what the robot has personally explored.

Output geometry: MERGED map's geometry (origin/extent/resolution).
Earlier we kept local geometry, which made the swarm contribution
cosmetic — a robot that hadn't physically driven east couldn't see
A's east discoveries even though /merged_map had them. The planner
then refused to plan to any frontier outside the local extent (goal
clamped to map edge → unknown → no path → robot frozen forever even
though merged knew the way). With merged geometry, the planner sees
the full swarm-discovered map; local readings override merged
wherever they exist (most-recent / on-robot data still wins).

Why this rather than astar_nav subscribing to /merged_map directly:

    The "let merged-map update my map" framing is what real robots need.
    On-robot, each platform maintains its own occupancy representation;
    contributions from peers arrive as messages over a wireless link and
    get reconciled into the local map. There is no centralised planner
    state. multirobot_map_merge in sim is just one source of those
    contributions; on a real swarm it could be replaced by peer-to-peer
    octomap-diff messages or a shared-key-frame matcher without changing
    this node or the planner.

Frame alignment: this node assumes both maps live in the same world
frame (in MuJoCo sim with GT-bootstrapped init poses, both /merged_map
and /{ns}/map_raw use the same global "map" frame). On real robots the
caller is responsible for ensuring the swarm-merge upstream has aligned
the maps before publishing /merged_map. If frames differ, this node
will produce a corrupted output — a deliberate failure rather than
silent miscompose.
"""
from __future__ import annotations
import array
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)


class MapAugmenter(Node):
    def __init__(self) -> None:
        super().__init__("map_augmenter")
        # Topic names are relative — launch puts the node in /{ns}, so
        # `map_raw` resolves to /{ns}/map_raw and `map` to /{ns}/map.
        # `merged_map_topic` is absolute by default so all robots share
        # one swarm view.
        self.declare_parameter("local_map_topic", "map_raw")
        self.declare_parameter("merged_map_topic", "/merged_map")
        self.declare_parameter("augmented_map_topic", "map")
        # Republish at the local map's update rate by default. We
        # passthrough on every local message; the timer is just a
        # heartbeat for the case where local stops updating but merged
        # keeps flowing (so the planner still gets the latest swarm
        # view through us).
        self.declare_parameter("heartbeat_rate_hz", 1.0)
        # Hard cap on output extent (each axis, m). Caps the union
        # bbox so a feedback loop with multirobot_map_merge can't
        # grow geometry unboundedly. demo3_mixed world ≈ 24 × 16 m,
        # 50 m is safe headroom.
        self.declare_parameter("max_extent_m", 50.0)
        self._max_extent_m = max(1.0, float(self.get_parameter("max_extent_m").value))

        local_topic = self.get_parameter("local_map_topic").value
        merged_topic = self.get_parameter("merged_map_topic").value
        out_topic = self.get_parameter("augmented_map_topic").value
        hb_rate = max(0.1, float(self.get_parameter("heartbeat_rate_hz").value))

        # Octomap and multirobot_map_merge both publish RELIABLE +
        # TRANSIENT_LOCAL — match or DDS silently drops messages.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        self._local_msg: OccupancyGrid | None = None
        self._merged_msg: OccupancyGrid | None = None
        self._merge_count = 0
        self._republish_count = 0

        self.create_subscription(OccupancyGrid, local_topic,
                                 self._on_local, map_qos)
        self.create_subscription(OccupancyGrid, merged_topic,
                                 self._on_merged, map_qos)
        self._pub = self.create_publisher(OccupancyGrid, out_topic, map_qos)
        self.create_timer(1.0 / hb_rate, self._heartbeat)
        self.get_logger().info(
            f"map_augmenter started: local={local_topic} merged={merged_topic} "
            f"→ {out_topic} (heartbeat={hb_rate:.1f} Hz)"
        )

    def _on_local(self, msg: OccupancyGrid) -> None:
        self._local_msg = msg
        self._publish_augmented()

    def _on_merged(self, msg: OccupancyGrid) -> None:
        self._merged_msg = msg
        # Don't republish on every merged update — wait for the next
        # local update or the heartbeat to drive cadence. Otherwise
        # downstream consumers see staircased timestamps.

    def _heartbeat(self) -> None:
        # Re-emit the last good augmented map even if no new local
        # arrived — gives downstream a fresh stamp every 1 s.
        if self._local_msg is not None:
            self._publish_augmented()

    def _publish_augmented(self) -> None:
        local = self._local_msg
        if local is None:
            return
        merged = self._merged_msg
        if merged is None:
            # No swarm data yet — pass local through unchanged so the
            # planner doesn't have to wait for swarm bootstrap.
            out = OccupancyGrid()
            out.header = local.header
            out.info = local.info
            out.data = local.data
            self._pub.publish(out)
            self._republish_count += 1
            return

        # Output geometry = UNION of local and merged extents. Earlier
        # we used merged.info directly, but multirobot_map_merge can
        # lag behind / clip the bounding box, so cells in the local
        # octomap that fell outside merged's extent disappeared from
        # the output. demo3_mixed 2026-04-25: A explored east to
        # x=11.9 m, merged was capped at x=10.35 m, A's planner had
        # NO data on cross_v_n (the wall A then drove into). The
        # union geometry guarantees neither map contributes "blind"
        # cells — wherever EITHER has data, output has data, with
        # local taking priority on overlap.
        lw = int(local.info.width)
        lh = int(local.info.height)
        lres = float(local.info.resolution)
        lox = float(local.info.origin.position.x)
        loy = float(local.info.origin.position.y)

        mw = int(merged.info.width)
        mh = int(merged.info.height)
        mres = float(merged.info.resolution)
        mox = float(merged.info.origin.position.x)
        moy = float(merged.info.origin.position.y)

        if lw < 1 or lh < 1 or mw < 1 or mh < 1 or lres <= 0 or mres <= 0:
            return

        local_arr = np.frombuffer(bytes(local.data), dtype=np.int8)
        merged_arr = np.frombuffer(bytes(merged.data), dtype=np.int8)
        if local_arr.size != lw * lh or merged_arr.size != mw * mh:
            self.get_logger().warn(
                "Map size mismatch: local has %d cells (expected %d), "
                "merged has %d cells (expected %d). Passing local through."
                % (local_arr.size, lw * lh, merged_arr.size, mw * mh)
            )
            self._pub.publish(local)
            return

        # Compute union bounding box. Output resolution = local's
        # (so existing safety params calibrated for octomap res still
        # apply); merged is resampled into this grid at the same res.
        # Both maps in MuJoCo sim use 0.05 m so this is a no-op
        # except for the geometric extent.
        #
        # HARD CAP on output extent to break a positive feedback loop
        # with multirobot_map_merge:
        #   1. augmenter outputs /{ns}/map at union(local, merged)
        #      extent
        #   2. map_merge subscribes to /{ns}/map (per its config), bbox
        #      = max of inputs
        #   3. /merged_map = bbox of (already-augmented) inputs, slightly
        #      larger due to cumulative warpAffine drift in
        #      multirobot_map_merge
        #   4. augmenter sees larger merged → output extent grows
        #   5. loop — observed 807×325 → 814×325 in 4 s (2026-04-25)
        # The cap stops step 4: even if merged grows past the world
        # bounds, augmenter output stays bounded. map_merge's bbox is
        # then dictated by the smallest input extent (capped) instead
        # of growing unbounded.
        # World size for demo3_mixed = 24 × 16 m centred near origin.
        # 50 × 50 m gives huge headroom for any reasonable scene.
        out_res = lres
        ux_min = min(lox, mox)
        uy_min = min(loy, moy)
        ux_max = max(lox + lw * lres, mox + mw * mres)
        uy_max = max(loy + lh * lres, moy + mh * mres)
        # Cap to world bounds
        max_extent_m = self._max_extent_m
        if (ux_max - ux_min) > max_extent_m:
            cx = 0.5 * (ux_min + ux_max)
            ux_min = cx - 0.5 * max_extent_m
            ux_max = cx + 0.5 * max_extent_m
        if (uy_max - uy_min) > max_extent_m:
            cy = 0.5 * (uy_min + uy_max)
            uy_min = cy - 0.5 * max_extent_m
            uy_max = cy + 0.5 * max_extent_m
        out_w = max(1, int(np.ceil((ux_max - ux_min) / out_res)))
        out_h = max(1, int(np.ceil((uy_max - uy_min) / out_res)))
        out_ox = ux_min
        out_oy = uy_min

        # Cell centres → world coords for every output cell.
        ogy_arr, ogx_arr = np.divmod(np.arange(out_w * out_h, dtype=np.int64), out_w)
        wx = out_ox + (ogx_arr.astype(np.float64) + 0.5) * out_res
        wy = out_oy + (ogy_arr.astype(np.float64) + 0.5) * out_res

        # Look up merged value at each output cell.
        mgx = np.floor((wx - mox) / mres).astype(np.int64)
        mgy = np.floor((wy - moy) / mres).astype(np.int64)
        in_merged = (mgx >= 0) & (mgx < mw) & (mgy >= 0) & (mgy < mh)
        midx = np.where(in_merged, mgy * mw + mgx, 0)
        merged_value = merged_arr[midx]

        # Start unknown; fill from merged where in-bounds.
        out_arr = np.full(out_w * out_h, -1, dtype=np.int8)
        out_arr[in_merged] = merged_value[in_merged]

        # Look up local value; override wherever local is known
        # (local takes priority — on-robot, freshest, fewer alignment
        # errors. Critical: when merged drops cells the local octomap
        # still has, this layer keeps them.)
        lgx = np.floor((wx - lox) / lres).astype(np.int64)
        lgy = np.floor((wy - loy) / lres).astype(np.int64)
        in_local = (lgx >= 0) & (lgx < lw) & (lgy >= 0) & (lgy < lh)
        lidx = np.where(in_local, lgy * lw + lgx, 0)
        local_value = local_arr[lidx]
        local_known = (local_value >= 0) & in_local
        if local_known.any():
            out_arr[local_known] = local_value[local_known]
            self._merge_count += 1

        out = OccupancyGrid()
        out.header = local.header  # Keep local frame_id + stamp
        out.info.resolution = out_res
        out.info.width = out_w
        out.info.height = out_h
        out.info.origin.position.x = out_ox
        out.info.origin.position.y = out_oy
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        # rclpy OccupancyGrid.data needs int8 sequence; array('b',...) is the
        # zero-copy fast path.
        out.data = array.array("b", out_arr.tobytes())
        self._pub.publish(out)
        self._republish_count += 1

        if self._republish_count % 10 == 0:
            n_total = out_w * out_h
            n_local_overrides = int(local_known.sum())
            n_unknown = int((out_arr < 0).sum())
            self.get_logger().info(
                "augment: union geom %dx%d origin=(%.2f,%.2f) "
                "(%d cells, %d local-override, %d unknown) | "
                "local %dx%d, merged %dx%d"
                % (out_w, out_h, out_ox, out_oy,
                   n_total, n_local_overrides, n_unknown,
                   lw, lh, mw, mh)
            )


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = MapAugmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
