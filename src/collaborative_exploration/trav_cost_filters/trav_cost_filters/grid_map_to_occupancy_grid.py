#!/usr/bin/env python3
"""Convert traversability layer of a GridMap to OccupancyGrid for Nav2.

Subscribes to a filtered GridMap topic (expects a 'trav_eth' layer
in [0, 1], where 1 = fully traversable, 0 = blocked).
Layer name is 'trav_eth' (not 'traversability') to avoid aliasing with the
pre-existing all-NaN traversability layer from elevation_mapping_cupy.

Publishes OccupancyGrid in Nav2 cost convention:
  0   = free  (trav >= free_threshold)
  100 = lethal (trav < lethal_threshold)
  <linear interpolation in between>

This gives Nav2 a traversability-weighted costmap layer (via the StaticLayer
or a custom costmap plugin that reads OccupancyGrid).

Persistent fixed-origin buffer: locks origin on first GridMap and projects
each rolling-window frame into the same world-fixed buffer. Prevents Nav2
StaticLayer from resizing the global costmap every frame.

Data layout note (empirically verified):
  GridMap publishes column-major with outer dim = column index (y-axis),
  inner dim = row index (x-axis). col 0 = max-y, row 0 = max-x.
  Correct OccupancyGrid transform: reshape(n_y, n_x)[::-1, ::-1]
  (both axes flip to go from max→min to min→max; no transpose needed).
"""

import math

import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid

from trav_cost_filters.occupancy_conversion import (
    apply_cliff_proximity_cost,
    apply_rectangular_workspace_mask,
    apply_slope_verified_ramp_override,
    grid_map_layer_to_world_array,
    project_rolling_grid_to_fixed_grid,
    stamp_free_disk,
    traversability_to_occupancy,
)


class GridMapToOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_to_occupancy_grid")

        self.input_topic = self.declare_parameter(
            "input_topic", "elevation_map_filtered"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "traversability_grid"
        ).value
        self.traversability_layer = self.declare_parameter(
            "traversability_layer", "trav_eth"
        ).value
        # trav >= free_threshold → cost 0 (free)
        self.free_threshold = self.declare_parameter("free_threshold", 0.7).value
        # trav < lethal_threshold → cost 100 (lethal)
        self.lethal_threshold = self.declare_parameter("lethal_threshold", 0.3).value
        self.seed_robot_footprint = self.declare_parameter(
            "seed_robot_footprint", True
        ).value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        self.robot_seed_radius_m = float(
            self.declare_parameter("robot_seed_radius_m", 0.65).value
        )
        self.seed_max_clear_cost = int(
            self.declare_parameter("seed_max_clear_cost", 50).value
        )
        # Inner "core" footprint disk that is ALWAYS cleared free, even over
        # lethal cells. The robot is physically standing on these cells, so any
        # lethal there is self-paint (the robot's own body/legs returning LiDAR
        # points the elevation map can't tell from an obstacle) or stale. The
        # outer seed disk (robot_seed_radius_m) stays conservative (only clears
        # ≤ seed_max_clear_cost) so it never erases a real wall the robot is
        # driving alongside. Keep the core ≤ the robot's circumscribed radius
        # (~0.40 m) so it can't clear a genuine obstacle the robot isn't on.
        self.robot_core_clear_radius_m = float(
            self.declare_parameter("robot_core_clear_radius_m", 0.40).value
        )
        # Persistent visited-corridor mask. The robot physically occupied every
        # cell along its trajectory, so those cells are GROUND-TRUTH traversable
        # forever — a robot cannot stand inside a wall. Without this, the
        # Mid-360 blind disk (~3.25 m) leaves the trail behind the robot as
        # UNKNOWN in the elevation map; with allow_unknown=false the CFPA2
        # reachability BFS then can't flow back along the trail and the reachable
        # component collapses to a local bubble (run23: 73/73 frontiers
        # "unreachable", robot circling +x). We OR the swept capsule between
        # consecutive poses into a persistent mask and force those cells FREE on
        # every publish, immune to the rolling-grid re-clearing them to unknown.
        # Radius = robot circumscribed half-extent + small margin; narrow on
        # purpose so it carves a connected centerline corridor, never a wall.
        self.visited_corridor_enabled = bool(
            self.declare_parameter("visited_corridor_enabled", True).value
        )
        self.visited_corridor_radius_m = float(
            self.declare_parameter("visited_corridor_radius_m", 0.45).value
        )
        self._visited_mask: np.ndarray | None = None
        self._visited_prev_cx: float | None = None
        self._visited_prev_cy: float | None = None
        self.ramp_override_enabled = bool(
            self.declare_parameter("ramp_override_enabled", False).value
        )
        self.slope_layer = str(self.declare_parameter("slope_layer", "slope").value)
        self.step_residual_layer = str(
            self.declare_parameter("step_residual_layer", "step_residual").value
        )
        self.ramp_min_slope_rad = float(
            self.declare_parameter("ramp_min_slope_rad", 0.13962634015954636).value
        )
        self.ramp_max_slope_rad = float(
            self.declare_parameter("ramp_max_slope_rad", 0.5235987755982988).value
        )
        self.ramp_max_step_residual_m = float(
            self.declare_parameter("ramp_max_step_residual_m", 0.06).value
        )
        # Elevation-based extra cost (encourages staying low). Off by default.
        # Cells with elevation > min_h get cost = clip((h-min)/(max-min)*99, 0, max_val).
        # The trav-derived cost and elevation cost are combined via max() so:
        #   - lethal trav stays lethal
        #   - free flat ground stays free (elevation_cost ≈ 0)
        #   - free-but-elevated cells (ramps, platforms) get the elevation cost,
        #     which the planner treats as expensive but traversable
        self.elevation_cost_enabled = bool(
            self.declare_parameter("elevation_cost_enabled", False).value
        )
        self.elevation_layer = str(
            self.declare_parameter("elevation_layer", "elevation").value
        )
        self.elevation_cost_min_h = float(
            self.declare_parameter("elevation_cost_min_h", 0.05).value
        )
        self.elevation_cost_max_h = float(
            self.declare_parameter("elevation_cost_max_h", 1.00).value
        )
        self.elevation_cost_max_value = int(
            self.declare_parameter("elevation_cost_max_value", 90).value
        )
        # Stability margin around sharp vertical discontinuities. This is not
        # obstacle detection; it raises the cost of known traversable cells near
        # cliff/platform edges where the robot support polygon can tip.
        self.cliff_proximity_cost_enabled = bool(
            self.declare_parameter("cliff_proximity_cost_enabled", False).value
        )
        self.cliff_step_layer = str(
            self.declare_parameter("cliff_step_layer", "step_height").value
        )
        self.cliff_proximity_radius_m = float(
            self.declare_parameter("cliff_proximity_radius_m", 0.25).value
        )
        self.cliff_step_threshold_m = float(
            self.declare_parameter("cliff_step_threshold_m", 0.30).value
        )
        self.cliff_step_saturation_m = float(
            self.declare_parameter("cliff_step_saturation_m", 0.45).value
        )
        self.cliff_proximity_cost_max_value = int(
            self.declare_parameter("cliff_proximity_cost_max_value", 90).value
        )
        # Upper-bound clearance (Miki et al. 2022, Sec. II-H). When the
        # ray-cast-derived upper_bound is well below the cell's elevation
        # reading, the elevation point came from an overhang above an
        # observed-clear floor (bridge underside, ceiling) — reclassify
        # such cells as free instead of letting them remain lethal.
        self.upper_bound_clearance_enabled = bool(
            self.declare_parameter("upper_bound_clearance_enabled", False).value
        )
        self.upper_bound_layer = str(
            self.declare_parameter("upper_bound_layer", "upper_bound").value
        )
        # If elevation - upper_bound exceeds this, the elevation reading is
        # treated as an overhang and the cell is cleared. 0.30m chosen so a
        # small Kalman fusion artefact (~0.1m) doesn't trip it.
        self.upper_bound_overhang_threshold_m = float(
            self.declare_parameter("upper_bound_overhang_threshold_m", 0.30).value
        )
        # Cost to assign to cells flagged as overhang-over-floor. 0 = free.
        self.upper_bound_clear_cost = int(
            self.declare_parameter("upper_bound_clear_cost", 0).value
        )
        self.fixed_grid_enabled = bool(
            self.declare_parameter("fixed_grid_enabled", False).value
        )
        self.fixed_origin_x = float(
            self.declare_parameter("fixed_origin_x", 0.0).value
        )
        self.fixed_origin_y = float(
            self.declare_parameter("fixed_origin_y", 0.0).value
        )
        self.fixed_width_cells = int(
            self.declare_parameter("fixed_width_cells", 0).value
        )
        self.fixed_height_cells = int(
            self.declare_parameter("fixed_height_cells", 0).value
        )
        self.unknown_clears_history = bool(
            self.declare_parameter("unknown_clears_history", False).value
        )
        self.occupied_cost_threshold = int(
            self.declare_parameter("occupied_cost_threshold", 100).value
        )
        self.free_cost_threshold = int(
            self.declare_parameter("free_cost_threshold", 30).value
        )
        self.occupied_confirm_hits = int(
            self.declare_parameter("occupied_confirm_hits", 2).value
        )
        self.occupied_clear_hits = int(
            self.declare_parameter("occupied_clear_hits", 0).value
        )
        self.occupied_hit_increment = int(
            self.declare_parameter("occupied_hit_increment", 1).value
        )
        self.free_hit_decrement = int(
            self.declare_parameter("free_hit_decrement", 1).value
        )
        self.max_hit_count = int(
            self.declare_parameter("max_hit_count", 8).value
        )
        self.workspace_mask_enabled = bool(
            self.declare_parameter("workspace_mask_enabled", False).value
        )
        self.workspace_min_x = float(
            self.declare_parameter("workspace_min_x", 0.0).value
        )
        self.workspace_max_x = float(
            self.declare_parameter("workspace_max_x", 0.0).value
        )
        self.workspace_min_y = float(
            self.declare_parameter("workspace_min_y", 0.0).value
        )
        self.workspace_max_y = float(
            self.declare_parameter("workspace_max_y", 0.0).value
        )
        self.workspace_wall_thickness_m = float(
            self.declare_parameter("workspace_wall_thickness_m", 0.0).value
        )

        self._fixed_cost: np.ndarray | None = None
        self._fixed_hits: np.ndarray | None = None
        self._fixed_resolution: float | None = None

        self.tf_buffer = Buffer() if self.seed_robot_footprint else None
        self.tf_listener = (
            TransformListener(self.tf_buffer, self)
            if self.tf_buffer is not None
            else None
        )

        # Publisher uses TRANSIENT_LOCAL so late subscribers (diag tools, RViz,
        # CFPA2 BFS) receive the last grid immediately on connection.
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, pub_qos)
        self.sub = self.create_subscription(
            GridMap, self.input_topic, self._on_map, sub_qos
        )

        # Persistent fixed-origin buffer state.
        self._buf: np.ndarray | None = None
        self._fixed_ox: float | None = None
        self._fixed_oy: float | None = None
        self._fixed_res: float | None = None
        self._fixed_fw: int | None = None   # width  (n_x / cols)
        self._fixed_fh: int | None = None   # height (n_y / rows)
        self._frame_id: str | None = None

        self.get_logger().info(
            f"grid_map_to_occupancy_grid: {self.input_topic} → {self.output_topic} "
            f"layer={self.traversability_layer} "
            f"free>={self.free_threshold} lethal<{self.lethal_threshold} "
            f"seed_robot_footprint={self.seed_robot_footprint} "
            f"robot_frame={self.robot_frame} radius={self.robot_seed_radius_m:.2f}m "
            f"ramp_override={self.ramp_override_enabled} "
            f"cliff_proximity_cost={self.cliff_proximity_cost_enabled} "
            f"fixed_grid={self.fixed_grid_enabled}"
        )

    def _layer_array(self, msg: GridMap, layer_name: str) -> np.ndarray | None:
        """Return a layer as a (n_y, n_x) array aligned with OccupancyGrid convention."""
        if layer_name not in msg.layers:
            return None
        n_y = int(round(msg.info.length_y / msg.info.resolution))
        n_x = int(round(msg.info.length_x / msg.info.resolution))
        expected = n_y * n_x
        layer_idx = list(msg.layers).index(layer_name)
        data = np.array(msg.data[layer_idx].data, dtype=np.float32)
        if data.size != expected:
            return None
        try:
            return grid_map_layer_to_world_array(data, height=n_y, width=n_x)
        except ValueError:
            return None

    def _on_map(self, msg: GridMap) -> None:
        if self.traversability_layer not in msg.layers:
            self.get_logger().warn(
                f"layer '{self.traversability_layer}' not in GridMap; "
                f"available: {list(msg.layers)}",
                throttle_duration_sec=5.0,
            )
            return

        layer_idx = list(msg.layers).index(self.traversability_layer)
        info = msg.info
        res = info.resolution

        # n_y = rows (y-axis / height), n_x = cols (x-axis / width)
        n_y = int(round(info.length_y / res))
        n_x = int(round(info.length_x / res))
        n_cells = n_y * n_x

        data_float = np.array(msg.data[layer_idx].data, dtype=np.float32)
        if data_float.size != n_cells:
            self.get_logger().warn(
                f"size mismatch: data={data_float.size} expected={n_cells}",
                throttle_duration_sec=5.0,
            )
            return

        # GridMap column-major: outer dim = column index (y-axis), col 0 = max-y;
        # inner dim = row index (x-axis), row 0 = max-x.
        # Flip both axes to produce OccupancyGrid layout where
        # arr[r, c] = world (ox + c*res, oy + r*res).
        trav = grid_map_layer_to_world_array(data_float, height=n_y, width=n_x)

        cost = traversability_to_occupancy(
            trav,
            free_threshold=float(self.free_threshold),
            lethal_threshold=float(self.lethal_threshold),
        )
        if self.ramp_override_enabled:
            slope = self._layer_array(msg, self.slope_layer)
            step_residual = self._layer_array(msg, self.step_residual_layer)
            if slope is not None and step_residual is not None:
                changed = apply_slope_verified_ramp_override(
                    cost,
                    slope=slope,
                    step_residual=step_residual,
                    min_slope_rad=float(self.ramp_min_slope_rad),
                    max_slope_rad=float(self.ramp_max_slope_rad),
                    max_step_residual_m=float(self.ramp_max_step_residual_m),
                )
                if changed > 0:
                    self.get_logger().debug(
                        f"slope-verified ramp override cleared {changed} cells"
                    )

        # Elevation-based cost: penalise high-z cells so the planner prefers
        # flat ground when reaching the same frontier doesn't require climbing.
        if self.elevation_cost_enabled:
            elev = self._layer_array(msg, self.elevation_layer)
            if elev is not None and elev.shape == cost.shape:
                h_min = self.elevation_cost_min_h
                h_max = self.elevation_cost_max_h
                v_max = self.elevation_cost_max_value
                denom = max(1e-6, h_max - h_min)
                with np.errstate(invalid="ignore"):
                    over = np.clip((elev - h_min) / denom, 0.0, 1.0)
                    h_cost = np.where(np.isfinite(elev), over * v_max, 0.0).astype(np.int16)
                # Combine: lethal trav stays lethal, unknown stays unknown,
                # everything else gets max(trav_cost, h_cost).
                valid = (cost >= 0)
                merged = np.where(valid, np.maximum(cost.astype(np.int16), h_cost), -1)
                cost = merged.astype(np.int8)

        # Overhang clearance via upper_bound (paper Sec. II-H). Applied
        # BEFORE cliff/proximity costs so a cell rescued as walk-under-bridge
        # isn't subsequently re-penalised by an apparent step at its edge
        # (the overhang itself produces a fake step gradient).
        if self.upper_bound_clearance_enabled:
            elev = self._layer_array(msg, self.elevation_layer)
            ubnd = self._layer_array(msg, self.upper_bound_layer)
            if (elev is not None and ubnd is not None
                    and elev.shape == cost.shape and ubnd.shape == cost.shape):
                with np.errstate(invalid="ignore"):
                    gap = elev - ubnd
                    overhang_mask = (
                        np.isfinite(elev) & np.isfinite(ubnd)
                        & (gap > self.upper_bound_overhang_threshold_m)
                    )
                if overhang_mask.any():
                    cost = np.where(
                        overhang_mask,
                        np.int8(self.upper_bound_clear_cost),
                        cost,
                    )
                    self.get_logger().debug(
                        f"upper_bound clearance: cleared "
                        f"{int(overhang_mask.sum())} overhang cells"
                    )

        if self.cliff_proximity_cost_enabled:
            step_height = self._layer_array(msg, self.cliff_step_layer)
            if step_height is not None and step_height.shape == cost.shape:
                changed = apply_cliff_proximity_cost(
                    cost,
                    step_height=step_height,
                    resolution=float(res),
                    proximity_radius_m=self.cliff_proximity_radius_m,
                    step_threshold_m=self.cliff_step_threshold_m,
                    step_saturation_m=self.cliff_step_saturation_m,
                    max_cost=self.cliff_proximity_cost_max_value,
                )
                if changed > 0:
                    self.get_logger().debug(
                        f"cliff proximity cost raised {changed} cells"
                    )

        # Rolling-window origin (bottom-left corner of current GridMap).
        roll_ox = info.pose.position.x - info.length_x / 2.0
        roll_oy = info.pose.position.y - info.length_y / 2.0

        # --- Persistent fixed-origin buffer ---
        # Lock the buffer dimensions and origin on the first frame; all
        # subsequent frames project their rolling-window data into this fixed
        # grid. Nav2 StaticLayer then sees a stable map origin and never has
        # to resize the global costmap.
        if self._buf is None:
            self._fixed_ox = roll_ox
            self._fixed_oy = roll_oy
            self._fixed_res = res
            self._fixed_fw = n_x
            self._fixed_fh = n_y
            self._frame_id = msg.header.frame_id
            self._buf = np.full((n_y, n_x), -1, dtype=np.int8)
            self.get_logger().info(
                f"Fixed-origin buffer initialised: "
                f"origin=({self._fixed_ox:.2f}, {self._fixed_oy:.2f}) "
                f"size={n_x}x{n_y} @ {res:.3f}m/cell"
            )

        fw, fh = self._fixed_fw, self._fixed_fh

        # Offset of current rolling window origin relative to fixed origin (in cells).
        dc = int(round((roll_ox - self._fixed_ox) / res))
        dr = int(round((roll_oy - self._fixed_oy) / res))

        # Compute the overlap between the fixed buffer and current rolling window.
        R_lo = max(0, dr);      R_hi = min(fh, dr + n_y)
        C_lo = max(0, dc);      C_hi = min(fw, dc + n_x)

        if R_hi > R_lo and C_hi > C_lo:
            r_lo = R_lo - dr;   r_hi = R_hi - dr
            c_lo = C_lo - dc;   c_hi = C_hi - dc

            rolling_slice = cost[r_lo:r_hi, c_lo:c_hi]
            valid = rolling_slice >= 0
            self._buf[R_lo:R_hi, C_lo:C_hi][valid] = rolling_slice[valid]

        # Build and publish OccupancyGrid from the fixed buffer.
        occ = OccupancyGrid()
        occ.header.stamp = msg.header.stamp
        occ.header.frame_id = self._frame_id
        occ.info.resolution = res

        if self.fixed_grid_enabled:
            # Hit-counting fixed grid (configurable origin/size, workspace mask).
            cost = self._update_fixed_grid(cost, info)
            occ.info.width = cost.shape[1]
            occ.info.height = cost.shape[0]
            occ.info.origin.position.x = self.fixed_origin_x
            occ.info.origin.position.y = self.fixed_origin_y
            occ.info.origin.position.z = 0.0
            occ.info.origin.orientation.w = 1.0
        else:
            # Default: first-frame-locked persistent buffer (simpler, proven stable).
            cost = self._buf
            occ.info.width = fw
            occ.info.height = fh
            occ.info.origin.position.x = self._fixed_ox
            occ.info.origin.position.y = self._fixed_oy
            occ.info.origin.position.z = 0.0
            occ.info.origin.orientation.w = 1.0

        self._seed_robot_footprint(cost, occ)
        occ.data = cost.flatten().tolist()

        self.pub.publish(occ)

    def _update_fixed_grid(self, rolling_cost: np.ndarray, info) -> np.ndarray:
        resolution = float(info.resolution)
        width = self.fixed_width_cells if self.fixed_width_cells > 0 else rolling_cost.shape[1]
        height = self.fixed_height_cells if self.fixed_height_cells > 0 else rolling_cost.shape[0]
        shape = (int(height), int(width))

        if (
            self._fixed_cost is None
            or self._fixed_hits is None
            or self._fixed_cost.shape != shape
            or self._fixed_resolution != resolution
        ):
            self._fixed_cost = np.full(shape, -1, dtype=np.int8)
            self._fixed_hits = np.zeros(shape, dtype=np.int16)
            self._fixed_resolution = resolution
            self.get_logger().info(
                f"fixed traversability grid initialized: "
                f"{width}x{height} res={resolution:.3f} "
                f"origin=({self.fixed_origin_x:.2f},{self.fixed_origin_y:.2f})"
            )

        rolling_origin_x = float(info.pose.position.x - info.length_x / 2.0)
        rolling_origin_y = float(info.pose.position.y - info.length_y / 2.0)
        changed = project_rolling_grid_to_fixed_grid(
            rolling_cost,
            self._fixed_cost,
            self._fixed_hits,
            rolling_origin_x=rolling_origin_x,
            rolling_origin_y=rolling_origin_y,
            fixed_origin_x=self.fixed_origin_x,
            fixed_origin_y=self.fixed_origin_y,
            resolution=resolution,
            unknown_clears_history=self.unknown_clears_history,
            occupied_cost_threshold=self.occupied_cost_threshold,
            free_cost_threshold=self.free_cost_threshold,
            occupied_confirm_hits=self.occupied_confirm_hits,
            occupied_clear_hits=self.occupied_clear_hits,
            occupied_hit_increment=self.occupied_hit_increment,
            free_hit_decrement=self.free_hit_decrement,
            max_hit_count=self.max_hit_count,
        )
        if changed > 0:
            self.get_logger().debug(f"fixed traversability grid updated {changed} cells")

        if self.workspace_mask_enabled:
            apply_rectangular_workspace_mask(
                self._fixed_cost,
                origin_x=self.fixed_origin_x,
                origin_y=self.fixed_origin_y,
                resolution=resolution,
                min_x=self.workspace_min_x,
                max_x=self.workspace_max_x,
                min_y=self.workspace_min_y,
                max_y=self.workspace_max_y,
                wall_thickness_m=self.workspace_wall_thickness_m,
            )

        return self._fixed_cost

    def _seed_robot_footprint(
        self,
        cost: np.ndarray,
        occ: OccupancyGrid,
    ) -> None:
        if not self.seed_robot_footprint or self.tf_buffer is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                occ.header.frame_id,
                self.robot_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"robot footprint seed skipped: cannot transform "
                f"{occ.header.frame_id} <- {self.robot_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        cx = float(tf.transform.translation.x)
        cy = float(tf.transform.translation.y)
        # Tier 1: inner core — unconditional clear (covers self-painted lethal
        # at the robot's own footprint so SmacHybrid's start pose is never in
        # collision). max_clear_cost 101 ⇒ clears every value incl. lethal 100.
        core_changed = 0
        if self.robot_core_clear_radius_m > 0.0:
            core_changed = stamp_free_disk(
                cost,
                origin_x=float(occ.info.origin.position.x),
                origin_y=float(occ.info.origin.position.y),
                resolution=float(occ.info.resolution),
                center_x=cx,
                center_y=cy,
                radius_m=self.robot_core_clear_radius_m,
                max_clear_cost=101,
            )
        # Tier 2: outer seed — UNKNOWN-ONLY. Fills the geometric ground blind
        # cone (unseen floor the robot stands on, cost=-1) but leaves every
        # scanned cell at its measured value. The Mid-360 (-15 deg tilt) sees
        # near walls from ~0.43 m even though flat ground first returns ~1.25 m,
        # so a plain free-disk out to the blind-cone radius would erase those
        # scannable near walls. unknown_only keeps them. radius is sized to the
        # measured ground blind cone (~1.25 m) + margin so genuinely-unexplored
        # unknown beyond the robot isn't falsely cleared free.
        changed = stamp_free_disk(
            cost,
            origin_x=float(occ.info.origin.position.x),
            origin_y=float(occ.info.origin.position.y),
            resolution=float(occ.info.resolution),
            center_x=cx,
            center_y=cy,
            radius_m=self.robot_seed_radius_m,
            max_clear_cost=self.seed_max_clear_cost,
            unknown_only=True,
        )
        changed += core_changed
        if changed > 0:
            self.get_logger().debug(
                f"seeded {changed} robot-footprint cells free at "
                f"({tf.transform.translation.x:.2f}, "
                f"{tf.transform.translation.y:.2f})"
            )

        # Persistent visited corridor: stamp the swept capsule between the
        # previous and current robot pose into a permanent mask, then force
        # every visited cell FREE. Ground truth — the robot was physically
        # there — so this can never erase a real wall; it only repairs the
        # blind-zone holes the rolling elevation map leaves behind the robot.
        if self.visited_corridor_enabled:
            self._stamp_visited_corridor(cost, occ, cx, cy)

    def _stamp_visited_corridor(
        self,
        cost: np.ndarray,
        occ: OccupancyGrid,
        cx: float,
        cy: float,
    ) -> None:
        if cost.ndim != 2:
            return
        if self._visited_mask is None or self._visited_mask.shape != cost.shape:
            self._visited_mask = np.zeros(cost.shape, dtype=bool)
            self._visited_prev_cx = None
            self._visited_prev_cy = None

        res = float(occ.info.resolution)
        ox = float(occ.info.origin.position.x)
        oy = float(occ.info.origin.position.y)
        if res <= 0.0:
            return
        rad_cells = max(1, int(round(self.visited_corridor_radius_m / res)))
        H, W = cost.shape

        def stamp_disk(wx: float, wy: float) -> None:
            cc = int((wx - ox) / res)
            cr = int((wy - oy) / res)
            r0 = max(0, cr - rad_cells)
            r1 = min(H - 1, cr + rad_cells)
            c0 = max(0, cc - rad_cells)
            c1 = min(W - 1, cc + rad_cells)
            if r0 > r1 or c0 > c1:
                return
            rr = np.arange(r0, r1 + 1)[:, None]
            ccc = np.arange(c0, c1 + 1)[None, :]
            disk = ((rr - cr) ** 2 + (ccc - cc) ** 2) <= (rad_cells * rad_cells)
            self._visited_mask[r0 : r1 + 1, c0 : c1 + 1] |= disk

        # Interpolate the segment from the previous pose so fast motion or a
        # slow publish rate can never leave a gap in the corridor.
        if self._visited_prev_cx is not None:
            px, py = self._visited_prev_cx, self._visited_prev_cy
            seg = math.hypot(cx - px, cy - py)
            n = max(1, int(seg / (res * 0.5)) + 1)
            for i in range(n + 1):
                t = i / n
                stamp_disk(px + (cx - px) * t, py + (cy - py) * t)
        else:
            stamp_disk(cx, cy)
        self._visited_prev_cx = cx
        self._visited_prev_cy = cy

        # Force visited cells free in the published grid (immune to the rolling
        # grid re-marking them unknown once the robot moves on).
        cost[self._visited_mask] = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridMapToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # During launch-managed SIGINT shutdown, rclpy can raise from a pending
        # subscription take after the context has already started shutting down.
        # Preserve real runtime failures while treating that path as clean exit.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
