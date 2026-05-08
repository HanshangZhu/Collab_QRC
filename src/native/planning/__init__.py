"""Core layered modules for default navigation (A* grid + local avoidance).

Additional helpers for native (non-ROS) integration:
  make_scan(hits_world, pose)  — convert 3D world-frame hits to 2D scan
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import DefaultNavConfig
from .state import GoalState, NavRuntimeState, RobotState, TickResult
from .coordinator import DefaultNavCoordinator


@dataclass
class Scan2D:
    """Minimal LaserScan-compatible object consumed by DefaultNavCoordinator."""
    angle_min: float
    angle_max: float
    angle_increment: float
    ranges: list[float]
    range_min: float = 0.05
    range_max: float = 20.0


def make_scan(hits_world: "np.ndarray", pose,
              n_bins: int = 360,
              range_max: float = 20.0) -> Scan2D:
    """Project world-frame 3D LiDAR hits onto 2D robot-local polar scan.

    hits_world: (N,3) float32 world-frame hit points from LiDARSensor.tick()
    pose: Pose dataclass (x,y,qw,qx,qy,qz)
    Returns Scan2D compatible with ScanAnalyzer + LocalPlanner.
    """
    import numpy as np

    yaw = math.atan2(
        2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz),
    )
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    rx, ry = pose.x, pose.y

    angle_inc = 2.0 * math.pi / n_bins
    ranges = [float("inf")] * n_bins

    if len(hits_world) > 0:
        dx = hits_world[:, 0] - rx
        dy = hits_world[:, 1] - ry
        # Rotate to robot local frame
        lx =  cos_y * dx + sin_y * dy
        ly = -sin_y * dx + cos_y * dy
        angles = np.arctan2(ly, lx)
        dists  = np.hypot(lx, ly)

        bins = ((angles + math.pi) / angle_inc).astype(int) % n_bins
        for i in range(len(bins)):
            b = bins[i]
            d = float(dists[i])
            if d < ranges[b]:
                ranges[b] = d

    # Replace inf with range_max
    ranges = [r if math.isfinite(r) else range_max for r in ranges]

    return Scan2D(
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=angle_inc,
        ranges=ranges,
        range_max=range_max,
    )


__all__ = [
    "GoalState",
    "NavRuntimeState",
    "DefaultNavConfig",
    "DefaultNavCoordinator",
    "RobotState",
    "TickResult",
    "Scan2D",
    "make_scan",
]
