#!/usr/bin/env python3
"""Convert grid_map elevation/traversability → OccupancyGrid on /robot/map.

Subscribes to a grid_map topic with an 'elevation' layer, classifies cells
by slope/step-height into traversable vs obstacle, and publishes a standard
OccupancyGrid so the downstream nav stack (CFPA2 + default_nav) works unchanged.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap


class ElevationToOccupancy(Node):
    def __init__(self):
        super().__init__('elevation_to_occupancy')

        self.declare_parameter('input_topic', '/robot/elevation_map')
        self.declare_parameter('output_topic', '/robot/map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('elevation_layer', 'elevation')
        self.declare_parameter('max_step_height', 0.08)    # 8cm step → obstacle
        self.declare_parameter('max_slope_rad', 0.35)      # ~20° slope → obstacle
        self.declare_parameter('unknown_threshold', -999.0) # cells below this = unknown

        input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.elevation_layer = self.get_parameter('elevation_layer').value
        self.max_step_height = self.get_parameter('max_step_height').value
        self.max_slope_rad = self.get_parameter('max_slope_rad').value
        self.unknown_threshold = self.get_parameter('unknown_threshold').value

        # Transient local to match nav stack expectations
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, map_qos)

        self.create_subscription(GridMap, input_topic, self._grid_map_cb, 10)

        self.get_logger().info(
            f'Elevation→Occupancy bridge: {input_topic} → {self.output_topic} '
            f'(step={self.max_step_height}m, slope={math.degrees(self.max_slope_rad):.0f}°)')

    def _grid_map_cb(self, msg: GridMap):
        # Find the elevation layer
        if self.elevation_layer not in msg.layers:
            self.get_logger().warn_once(
                f'Layer "{self.elevation_layer}" not in grid_map (available: {msg.layers})')
            return

        layer_idx = msg.layers.index(self.elevation_layer)
        cols = msg.info.length_x
        rows = msg.info.length_y
        resolution = msg.info.resolution

        # grid_map data is column-major
        n_cols = int(round(cols / resolution))
        n_rows = int(round(rows / resolution))
        data = np.array(msg.data[layer_idx].data, dtype=np.float32)

        if len(data) != n_cols * n_rows:
            self.get_logger().warn_once(
                f'Data size mismatch: got {len(data)}, expected {n_cols}x{n_rows}={n_cols*n_rows}')
            return

        elevation = data.reshape((n_rows, n_cols))

        # Classify cells
        occ_grid = np.full((n_rows, n_cols), -1, dtype=np.int8)  # default unknown

        valid = np.isfinite(elevation) & (elevation > self.unknown_threshold)

        # Compute local slope via gradient
        if np.sum(valid) > 10:
            grad_x = np.gradient(elevation, resolution, axis=1)
            grad_y = np.gradient(elevation, resolution, axis=0)
            slope = np.arctan(np.sqrt(grad_x**2 + grad_y**2))

            # Compute step height (max elevation diff in 3x3 neighborhood)
            from scipy.ndimage import maximum_filter, minimum_filter
            local_max = maximum_filter(elevation, size=3)
            local_min = minimum_filter(elevation, size=3)
            step_height = local_max - local_min

            # Classify
            traversable = valid & (slope < self.max_slope_rad) & (step_height < self.max_step_height)
            obstacle = valid & (~traversable)

            occ_grid[traversable] = 0    # free
            occ_grid[obstacle] = 100     # occupied

        # Build OccupancyGrid
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = self.frame_id
        og.info.resolution = float(resolution)
        og.info.width = n_cols
        og.info.height = n_rows
        og.info.origin.position.x = msg.info.pose.position.x - cols / 2.0
        og.info.origin.position.y = msg.info.pose.position.y - rows / 2.0
        og.info.origin.orientation.w = 1.0
        # Flatten row-major for OccupancyGrid
        og.data = occ_grid.flatten().tolist()
        self.map_pub.publish(og)


def main(args=None):
    rclpy.init(args=args)
    node = ElevationToOccupancy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
