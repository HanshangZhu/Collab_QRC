"""SE2 pose registry — replaces TF2 for single-robot standalone stack."""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np

Pose2D = Tuple[float, float, float]  # (x, y, yaw)


class TFManager:
    """Thread-unsafe SE2 pose store.  Single-process; no locking needed."""

    def __init__(self) -> None:
        self._poses: dict[str, Tuple[Pose2D, float]] = {}  # name → (pose, timestamp)

    def update(self, frame: str, pose: Pose2D, t: Optional[float] = None) -> None:
        self._poses[frame] = (pose, t if t is not None else time.monotonic())

    def get(self, frame: str) -> Optional[Pose2D]:
        entry = self._poses.get(frame)
        return entry[0] if entry else None

    def age_sec(self, frame: str) -> float:
        entry = self._poses.get(frame)
        if entry is None:
            return float("inf")
        return time.monotonic() - entry[1]

    # ── Utility ──────────────────────────────────────────────────────────────

    @staticmethod
    def distance(a: Pose2D, b: Pose2D) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @staticmethod
    def angle_diff(a: float, b: float) -> float:
        """Signed angle difference b - a, wrapped to [-π, π]."""
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    @staticmethod
    def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)


TF = TFManager()
