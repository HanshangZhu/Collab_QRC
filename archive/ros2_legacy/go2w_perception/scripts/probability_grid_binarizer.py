#!/usr/bin/env python3
"""Discretize probabilistic OccupancyGrid values into {-1, 0, 100}.

Designed for Cartographer's occupancy_grid_node output, which publishes
probabilities in [0, 100] plus -1 for unknown. Downstream planners in this
repo generally expect exact free/occupied values rather than soft probabilities.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class GridMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str


def _neighbor_sum(mask: np.ndarray) -> np.ndarray:
    """Count 8-connected neighbors of True cells using zero-padded convolution."""
    padded = np.pad(mask.astype(np.int16), 1, "constant")
    out = np.zeros(mask.shape, dtype=np.int16)
    for dr in range(-1, 2):
        for dc in range(-1, 2):
            if dr == 0 and dc == 0:
                continue
            out += padded[1 + dr : 1 + dr + mask.shape[0], 1 + dc : 1 + dc + mask.shape[1]]
    return out


def _remove_small_occupied_components(grid: np.ndarray, min_cells: int) -> np.ndarray:
    """Remove occupied (100) connected components smaller than *min_cells*."""
    if not np.any(grid == 100):
        return grid

    h, w = grid.shape
    visited = np.zeros((h, w), dtype=np.bool_)
    result = grid.copy()

    for r in range(h):
        for c in range(w):
            if visited[r, c] or grid[r, c] != 100:
                continue
            # BFS to find connected component
            queue = deque()
            queue.append((r, c))
            visited[r, c] = True
            component = []
            while queue:
                cr, cc = queue.popleft()
                component.append((cr, cc))
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = cr + dr, cc + dc
                        if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and grid[nr, nc] == 100:
                            visited[nr, nc] = True
                            queue.append((nr, nc))

            if len(component) < min_cells:
                for cr, cc in component:
                    result[cr, cc] = -1  # Mark as unknown

    return result


def _fill_single_cell_holes(grid: np.ndarray, neighbor_threshold: int) -> np.ndarray:
    """Fill unknown cells surrounded by enough occupied neighbors."""
    occ_neighbors = _neighbor_sum(grid == 100)
    result = grid.copy()
    result[(grid == -1) & (occ_neighbors >= neighbor_threshold)] = 100
    return result


def discretize_probability_grid(
    values: np.ndarray,
    free_threshold: int,
    occupied_threshold: int,
    previous_grid: np.ndarray | None,
    min_occupied_component_cells: int,
    fill_holes: bool,
    hole_neighbor_threshold: int,
) -> np.ndarray:
    """Convert probabilistic grid to discrete {-1, 0, 100}."""
    out = np.full(values.shape, -1, dtype=np.int8)

    # Cells with probability == -1 stay unknown
    known = ~np.isin(values, [-1])
    out[known & (values <= free_threshold)] = 0
    out[known & (values >= occupied_threshold)] = 100

    if min_occupied_component_cells > 1:
        out = _remove_small_occupied_components(out, min_occupied_component_cells)

    if fill_holes:
        out = _fill_single_cell_holes(out, hole_neighbor_threshold)

    return out


class ProbabilityGridBinarizer(Node):
    def __init__(self) -> None:
        super().__init__("probability_grid_binarizer")

        self.declare_parameter("input_topic", "/map_prob")
        self.declare_parameter("output_topic", "/map")
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("min_occupied_component_cells", 3)
        self.declare_parameter("fill_holes", True)
        self.declare_parameter("hole_neighbor_threshold", 7)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.min_occupied_component_cells = int(self.get_parameter("min_occupied_component_cells").value)
        self.fill_holes = bool(self.get_parameter("fill_holes").value)
        self.hole_neighbor_threshold = int(self.get_parameter("hole_neighbor_threshold").value)

        if self.free_threshold >= self.occupied_threshold:
            raise ValueError("free_threshold must be < occupied_threshold")
        if input_topic == output_topic:
            raise ValueError("input_topic and output_topic must differ")

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub = self.create_subscription(OccupancyGrid, input_topic, self._on_map, map_qos)
        self._pub = self.create_publisher(OccupancyGrid, output_topic, map_qos)
        self._previous_grid = None
        self._previous_meta = None

        self.get_logger().info(
            f"ProbabilityGridBinarizer started: {input_topic} -> {output_topic}"
            f" (free<={self.free_threshold}, occupied>={self.occupied_threshold}"
            f", min_occ_component={self.min_occupied_component_cells}"
            f", fill_holes={self.fill_holes}"
            f", hole_neighbors>={self.hole_neighbor_threshold})"
        )

    @staticmethod
    def _meta_from_msg(msg: OccupancyGrid) -> GridMeta:
        return GridMeta(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            frame_id=str(msg.header.frame_id),
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        meta = self._meta_from_msg(msg)

        if self._previous_meta is not None and meta != self._previous_meta:
            self.get_logger().info(
                f"Map metadata changed; resetting hysteresis cache ("
                f"{meta.width}x{meta.height} -> "
                f"{meta.width}x{meta.height}"
                f", res {meta.resolution:.3f})"
            )
            self._previous_grid = None

        values = np.array(msg.data, dtype=np.int16).reshape(meta.height, meta.width)

        result = discretize_probability_grid(
            values,
            self.free_threshold,
            self.occupied_threshold,
            self._previous_grid,
            self.min_occupied_component_cells,
            self.fill_holes,
            self.hole_neighbor_threshold,
        )

        out_msg = OccupancyGrid()
        out_msg.header = msg.header
        out_msg.info = msg.info
        out_msg.data = result.reshape(-1).astype(np.int8).tolist()
        self._pub.publish(out_msg)

        self._previous_grid = result.copy()
        self._previous_meta = meta


def main():
    rclpy.init()
    node = ProbabilityGridBinarizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
