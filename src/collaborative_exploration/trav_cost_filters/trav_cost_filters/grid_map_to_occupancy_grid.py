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
"""

import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid

from trav_cost_filters.occupancy_conversion import (
    apply_slope_verified_ramp_override,
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

        self.get_logger().info(
            f"grid_map_to_occupancy_grid: {self.input_topic} → {self.output_topic} "
            f"layer={self.traversability_layer} "
            f"free>={self.free_threshold} lethal<{self.lethal_threshold} "
            f"seed_robot_footprint={self.seed_robot_footprint} "
            f"robot_frame={self.robot_frame} radius={self.robot_seed_radius_m:.2f}m "
            f"ramp_override={self.ramp_override_enabled}"
        )

    def _layer_array(self, msg: GridMap, layer_name: str) -> np.ndarray | None:
        if layer_name not in msg.layers:
            return None
        rows = int(round(msg.info.length_y / msg.info.resolution))
        cols = int(round(msg.info.length_x / msg.info.resolution))
        expected = rows * cols
        layer_idx = list(msg.layers).index(layer_name)
        data = np.array(msg.data[layer_idx].data, dtype=np.float32)
        if data.size != expected:
            return None
        return data.reshape(cols, rows).T

    def _on_map(self, msg: GridMap) -> None:
        if self.traversability_layer not in msg.layers:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f"layer '{self.traversability_layer}' not in GridMap; "
                f"available: {list(msg.layers)}"
            )
            return

        layer_idx = list(msg.layers).index(self.traversability_layer)
        info = msg.info

        rows = info.length_y / info.resolution
        cols = info.length_x / info.resolution
        n_cells = int(round(rows)) * int(round(cols))

        data_float = np.array(msg.data[layer_idx].data, dtype=np.float32)

        if data_float.size != n_cells:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f"size mismatch: data={data_float.size} expected={n_cells}"
            )
            return

        # Reshape: grid_map stores data column-major (x varies first for each column).
        # OccupancyGrid is row-major (x = column, y = row in grid_map convention).
        # grid_map cell (row i, col j) → index i*cols + j in OccupancyGrid.
        n_rows = int(round(rows))
        n_cols = int(round(cols))

        trav = data_float.reshape(n_cols, n_rows)  # (cols, rows) column-major
        trav = trav.T  # → (rows, cols) row-major for OccupancyGrid

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

        occ = OccupancyGrid()
        occ.header.stamp = msg.header.stamp
        occ.header.frame_id = msg.header.frame_id
        occ.info.resolution = info.resolution
        occ.info.width = n_cols
        occ.info.height = n_rows
        # grid_map origin is centre; OccupancyGrid origin is bottom-left corner.
        occ.info.origin.position.x = (
            info.pose.position.x - info.length_x / 2.0
        )
        occ.info.origin.position.y = (
            info.pose.position.y - info.length_y / 2.0
        )
        occ.info.origin.position.z = info.pose.position.z
        occ.info.origin.orientation = info.pose.orientation
        self._seed_robot_footprint(cost, occ)
        occ.data = cost.flatten().tolist()

        self.pub.publish(occ)

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
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f"robot footprint seed skipped: cannot transform "
                f"{occ.header.frame_id} <- {self.robot_frame}: {exc}",
            )
            return

        changed = stamp_free_disk(
            cost,
            origin_x=float(occ.info.origin.position.x),
            origin_y=float(occ.info.origin.position.y),
            resolution=float(occ.info.resolution),
            center_x=float(tf.transform.translation.x),
            center_y=float(tf.transform.translation.y),
            radius_m=self.robot_seed_radius_m,
            max_clear_cost=self.seed_max_clear_cost,
        )
        if changed > 0:
            self.get_logger().debug(
                f"seeded {changed} robot-footprint cells free at "
                f"({tf.transform.translation.x:.2f}, "
                f"{tf.transform.translation.y:.2f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridMapToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
