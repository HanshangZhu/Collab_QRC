#!/usr/bin/env python3
"""Frontier detection → 3D vertical cylinder markers.

Subscribes to the 2D projected occupancy grid (/robot/map from octomap_server),
runs the same BFS cluster-centroid frontier extraction as CFPA2, and publishes
the results as vertical CYLINDER markers so they are visible in the 3D octomap
RViz view alongside the voxel grid.

Topics
------
  Subscribes : /robot/map  (nav_msgs/OccupancyGrid)
  Publishes  : /frontier_cylinders  (visualization_msgs/MarkerArray)
"""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class FrontierMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_3d_markers")

        # --- tunable params ---
        self.declare_parameter("map_topic",          "/map")
        self.declare_parameter("marker_topic",       "/frontier_cylinders")
        self.declare_parameter("frame_id",           "map")
        # Standard OccupancyGrid: free=0, occupied=100, unknown=-1
        self.declare_parameter("free_threshold",     0)    # cells <= this are free
        self.declare_parameter("unknown_value",      -1)
        self.declare_parameter("occ_threshold",      50)   # cells >= this are obstacles
        self.declare_parameter("frontier_stride",    2)
        self.declare_parameter("min_cluster_area_m2", 0.5)
        self.declare_parameter("obstacle_clearance_m", 0.15)
        self.declare_parameter("max_frontiers",      180)
        # display: top-N cluster centroids, min distance apart
        self.declare_parameter("display_top_n",      5)
        self.declare_parameter("display_min_dist_m", 0.5)
        # utility scoring: info gain = unknown cells within this radius
        self.declare_parameter("info_gain_radius_m", 1.5)
        # cylinder appearance
        self.declare_parameter("cylinder_height",    0.8)   # metres tall
        self.declare_parameter("cylinder_radius",    0.12)  # metres radius
        self.declare_parameter("cylinder_z_base",    0.0)   # floor z in map frame
        self.declare_parameter("color_r",            0.0)
        self.declare_parameter("color_g",            1.0)
        self.declare_parameter("color_b",            0.3)
        self.declare_parameter("color_a",            0.75)

        map_topic        = self.get_parameter("map_topic").value
        marker_topic     = self.get_parameter("marker_topic").value
        self.frame_id    = self.get_parameter("frame_id").value
        self.free_thresh = int(self.get_parameter("free_threshold").value)
        self.unk_val     = int(self.get_parameter("unknown_value").value)
        self.occ_thresh  = int(self.get_parameter("occ_threshold").value)
        self.stride      = max(1, int(self.get_parameter("frontier_stride").value))
        self.min_area    = float(self.get_parameter("min_cluster_area_m2").value)
        self.clearance_m = float(self.get_parameter("obstacle_clearance_m").value)
        self.max_fronts  = max(10, int(self.get_parameter("max_frontiers").value))
        self.top_n       = max(1, int(self.get_parameter("display_top_n").value))
        self.min_dist    = float(self.get_parameter("display_min_dist_m").value)
        self.ig_radius   = float(self.get_parameter("info_gain_radius_m").value)
        self.cyl_h       = float(self.get_parameter("cylinder_height").value)
        self.cyl_r       = float(self.get_parameter("cylinder_radius").value)
        self.cyl_z_base  = float(self.get_parameter("cylinder_z_base").value)
        self.color       = (
            float(self.get_parameter("color_r").value),
            float(self.get_parameter("color_g").value),
            float(self.get_parameter("color_b").value),
            float(self.get_parameter("color_a").value),
        )

        # Transient local QoS — matches Cartographer occupancy grid node
        map_qos = QoSProfile(depth=5)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

        self.get_logger().info(
            f"frontier_3d_markers: {map_topic} → {marker_topic} "
            f"(stride={self.stride}, min_area={self.min_area}m², "
            f"clearance={self.clearance_m}m)"
        )

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _idx(gx: int, gy: int, w: int) -> int:
        return gy * w + gx

    def _is_free(self, data, idx: int) -> bool:
        v = data[idx]
        return v >= 0 and v <= self.free_thresh

    def _is_unknown(self, data, idx: int) -> bool:
        return data[idx] == self.unk_val

    def _clear_of_obstacles(
        self, data, gx: int, gy: int, w: int, h: int, r: int
    ) -> bool:
        """Return True iff no occupied cell within radius r of (gx,gy)."""
        if r <= 0:
            return True
        r2 = r * r
        for dy in range(-r, r + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                return False
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= w:
                    return False
                if data[self._idx(nx, ny, w)] >= self.occ_thresh:
                    return False
        return True

    def _grid_to_world(self, msg: OccupancyGrid, gx: int, gy: int):
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        return (ox + (gx + 0.5) * res, oy + (gy + 0.5) * res)

    # ------------------------------------------------------------------ #
    #  Frontier extraction  (CFPA2 _extract_frontiers_py logic)           #
    # ------------------------------------------------------------------ #

    def _info_gain(self, data, cx_grid, cy_grid, w, h, r_cells):
        """Count unknown cells within r_cells of (cx_grid, cy_grid)."""
        cx, cy = int(round(cx_grid)), int(round(cy_grid))
        r2 = r_cells * r_cells
        count = 0
        for dy in range(-r_cells, r_cells + 1):
            ny = cy + dy
            if ny < 0 or ny >= h:
                continue
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = cx + dx
                if nx < 0 or nx >= w:
                    continue
                if self._is_unknown(data, self._idx(nx, ny, w)):
                    count += 1
        return count

    def _extract_frontier_clusters(self, msg: OccupancyGrid):
        """Return list of (centroid_x, centroid_y, utility) sorted by utility descending."""
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = max(1e-6, float(msg.info.resolution))
        data = msg.data
        ig_r_cells = int(math.ceil(self.ig_radius / res))

        N8 = ((1, 0), (-1, 0), (0, 1), (0, -1),
               (1, 1), (-1, 1), (1, -1), (-1, -1))

        # --- Pass 1: mark frontier cells (free + adjacent to unknown) ---
        frontier_mask = bytearray(w * h)
        frontier_cells = []
        for gy in range(1, h - 1):
            row = gy * w
            for gx in range(1, w - 1):
                idx = row + gx
                if not self._is_free(data, idx):
                    continue
                for dx, dy in N8:
                    if self._is_unknown(data, (gy + dy) * w + (gx + dx)):
                        frontier_mask[idx] = 1
                        frontier_cells.append((gx, gy))
                        break

        if not frontier_cells:
            return []

        # --- Pass 2: BFS cluster → centroid + info gain utility ---
        visited = bytearray(w * h)
        clusters = []  # (cx_world, cy_world, utility)

        for seed_x, seed_y in frontier_cells:
            seed_idx = self._idx(seed_x, seed_y, w)
            if visited[seed_idx] or not frontier_mask[seed_idx]:
                continue

            visited[seed_idx] = 1
            q = deque([(seed_x, seed_y)])
            component = []

            while q:
                cx, cy = q.popleft()
                component.append((cx, cy))
                for dx, dy in N8:
                    nx, ny = cx + dx, cy + dy
                    if nx <= 0 or ny <= 0 or nx >= w - 1 or ny >= h - 1:
                        continue
                    nidx = self._idx(nx, ny, w)
                    if visited[nidx] or not frontier_mask[nidx]:
                        continue
                    visited[nidx] = 1
                    q.append((nx, ny))

            area_m2 = len(component) * res * res
            if area_m2 < self.min_area:
                continue

            # Cluster centroid in grid coords
            n = len(component)
            cx_grid = sum(gx for gx, gy in component) / n
            cy_grid = sum(gy for gx, gy in component) / n

            # Utility = info gain (unknown cells within radius of centroid)
            utility = self._info_gain(data, cx_grid, cy_grid, w, h, ig_r_cells)

            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            cx_w = ox + (cx_grid + 0.5) * res
            cy_w = oy + (cy_grid + 0.5) * res
            clusters.append((cx_w, cy_w, utility))

        # Sort by utility descending (highest info gain first)
        clusters.sort(key=lambda c: c[2], reverse=True)
        return clusters

    # ------------------------------------------------------------------ #
    #  Map callback → detect → publish markers                            #
    # ------------------------------------------------------------------ #

    def _map_cb(self, msg: OccupancyGrid) -> None:
        data = msg.data
        total = len(data)

        # Diagnostic log every ~5 seconds
        now = time.monotonic()
        if not hasattr(self, '_last_diag') or now - self._last_diag > 5.0:
            self._last_diag = now
            free = sum(1 for v in data if 0 <= v <= self.free_thresh)
            unk  = sum(1 for v in data if v == self.unk_val)
            occ  = sum(1 for v in data if v >= self.occ_thresh)
            other = total - free - unk - occ
            self.get_logger().info(
                f"MAP {msg.info.width}x{msg.info.height} "
                f"free(<={self.free_thresh})={free} unk(=={self.unk_val})={unk} "
                f"occ(>={self.occ_thresh})={occ} other={other}"
            )

        clusters = self._extract_frontier_clusters(msg)

        # --- Display logic: pick top N clusters, each >= min_dist apart ---
        displayed = []  # list of (cx, cy, n_cells)
        for cx, cy, n in clusters:
            too_close = False
            for dx, dy, _ in displayed:
                if math.hypot(cx - dx, cy - dy) < self.min_dist:
                    too_close = True
                    break
            if too_close:
                continue
            displayed.append((cx, cy, n))
            if len(displayed) >= self.top_n:
                break

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.frame_id or msg.header.frame_id or "map"
        cyl_z_centre = self.cyl_z_base + self.cyl_h / 2.0
        r, g, b, a = self.color

        # Delete all previous markers first
        del_marker = Marker()
        del_marker.header.stamp = stamp
        del_marker.header.frame_id = frame
        del_marker.ns = "frontiers"
        del_marker.id = 0
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        for i, (wx, wy, utility) in enumerate(displayed):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.ns = "frontiers"
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = cyl_z_centre   # centre of cylinder
            m.pose.orientation.w = 1.0          # vertical (Z-axis aligned)

            m.scale.x = self.cyl_r * 2.0        # diameter
            m.scale.y = self.cyl_r * 2.0
            m.scale.z = self.cyl_h

            if i == 0:
                # Top-1 frontier (highest utility) → red, taller, wider
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.9
                m.scale.x = self.cyl_r * 3.0
                m.scale.y = self.cyl_r * 3.0
                m.scale.z = self.cyl_h * 1.5
                m.pose.position.z = self.cyl_z_base + (self.cyl_h * 1.5) / 2.0
            else:
                m.color.r = r
                m.color.g = g
                m.color.b = b
                m.color.a = a

            m.lifetime.sec = 3   # auto-expire if map stops updating
            ma.markers.append(m)

        self.marker_pub.publish(ma)
        if not hasattr(self, '_last_count_log') or now - self._last_count_log > 5.0:
            self._last_count_log = now
            self.get_logger().info(
                f"Clusters: {len(clusters)} total → {len(displayed)} displayed "
                f"(top {self.top_n}, min_dist={self.min_dist}m)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
