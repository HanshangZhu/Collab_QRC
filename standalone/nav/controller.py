"""Pure-pursuit controller — replaces Nav2 MPPI + DiffDrive motion model.

Tracks a path (list of (x, y) waypoints) using pure-pursuit lookahead.
Heading error is corrected with a P-gain on angular velocity.

Output: (vx, wz) in robot frame — forwarded to the hybrid cmd router.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

Waypoint = Tuple[float, float]
Path = List[Waypoint]
Pose2D = Tuple[float, float, float]  # (x, y, yaw)


class PurePursuitController:
    """Pure-pursuit with heading correction for SE2 diff-drive / wheel mode."""

    def __init__(
        self,
        *,
        lookahead_m: float = 0.8,
        max_vx: float = 0.5,
        max_wz: float = 1.2,
        goal_tolerance_m: float = 0.35,
        heading_gain: float = 2.0,
        min_vx_while_turning: float = 0.05,
    ) -> None:
        self._lookahead = lookahead_m
        self._max_vx = max_vx
        self._max_wz = max_wz
        self._goal_tol = goal_tolerance_m
        self._k_heading = heading_gain
        self._min_vx = min_vx_while_turning

        self._path: Path = []
        self._target_idx: int = 0
        self._goal_reached: bool = True

    # ── Path management ───────────────────────────────────────────────────────

    def set_path(self, path: Path) -> None:
        """Set a new path (list of (x, y) in world frame)."""
        if not path:
            self._path = []
            self._goal_reached = True
            return
        self._path = path
        self._target_idx = 0
        self._goal_reached = False

    def clear(self) -> None:
        self._path = []
        self._goal_reached = True

    @property
    def goal_reached(self) -> bool:
        return self._goal_reached

    @property
    def has_path(self) -> bool:
        return bool(self._path)

    # ── Control ───────────────────────────────────────────────────────────────

    def compute(self, pose: Pose2D) -> Tuple[float, float]:
        """Return (vx, wz) for the current robot pose.

        Returns (0, 0) when no path or goal reached.
        """
        if self._goal_reached or not self._path:
            return 0.0, 0.0

        rx, ry, ryaw = pose
        goal = self._path[-1]

        # Check final goal reached
        d2g = math.hypot(goal[0] - rx, goal[1] - ry)
        if d2g <= self._goal_tol:
            self._goal_reached = True
            return 0.0, 0.0

        # Advance target index along path
        while self._target_idx < len(self._path) - 1:
            tx, ty = self._path[self._target_idx]
            if math.hypot(tx - rx, ty - ry) > self._lookahead:
                break
            self._target_idx += 1

        tx, ty = self._path[self._target_idx]

        # Angle to lookahead target in world frame
        desired_yaw = math.atan2(ty - ry, tx - rx)
        heading_err = _angle_diff(desired_yaw, ryaw)

        # Angular velocity proportional to heading error
        wz = float(_clamp(self._k_heading * heading_err, -self._max_wz, self._max_wz))

        # Forward velocity: reduce when turning hard
        turn_ratio = abs(wz) / self._max_wz  # 0..1
        vx = self._max_vx * max(1.0 - turn_ratio, self._min_vx / self._max_vx)
        vx = float(_clamp(vx, 0.0, self._max_vx))

        return vx, wz


def _angle_diff(target: float, current: float) -> float:
    """Signed angle target - current, wrapped to [-π, π]."""
    d = target - current
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
