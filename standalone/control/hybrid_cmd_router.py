"""Hybrid cmd router — ported from go2w_hybrid_cmd_router.py (rclpy stripped).

Decides whether to use wheel mode or legged mode based on the requested
cmd_vel. In the standalone stack:
  - wheel mode  → forwards (vx, wz) to LegController
  - legged mode → passes zero cmd_vel (legs hold standing pose, wheels freewheel)
  - idle        → same as legged

The bus topics used match the originals so the rest of the stack is unchanged.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Tuple

from ..core.bus import BUS
from ..core.ros_compat import Twist, String


@dataclass
class MotionThresholds:
    idle_linear: float
    idle_lateral: float
    idle_angular: float
    wheel_linear: float
    wheel_lateral: float
    wheel_angular: float
    wheel_curvature: float


class HybridCmdRouter:
    """Stateful wheel/legged mode mux.  Call tick() on each nav update."""

    def __init__(
        self,
        *,
        idle_linear_threshold: float = 0.02,
        idle_lateral_threshold: float = 0.02,
        idle_angular_threshold: float = 0.05,
        wheel_linear_threshold: float = 0.18,
        wheel_lateral_threshold: float = 0.05,
        wheel_angular_threshold: float = 0.20,
        wheel_curvature_threshold: float = 0.45,
        wheel_mode_hold_sec: float = 0.6,
        legged_mode_hold_sec: float = 0.6,
        legged_override_curvature: float = 1.0,
        wheel_engage_sustain_sec: float = 0.5,
        cmd_timeout_sec: float = 0.50,
    ) -> None:
        self.thresholds = MotionThresholds(
            idle_linear=idle_linear_threshold,
            idle_lateral=idle_lateral_threshold,
            idle_angular=idle_angular_threshold,
            wheel_linear=wheel_linear_threshold,
            wheel_lateral=wheel_lateral_threshold,
            wheel_angular=wheel_angular_threshold,
            wheel_curvature=wheel_curvature_threshold,
        )
        self.mode_hold_sec = {"wheel": wheel_mode_hold_sec,
                              "legged": legged_mode_hold_sec}
        self.legged_override_curvature = legged_override_curvature
        self.wheel_engage_sustain_sec = wheel_engage_sustain_sec
        self.cmd_timeout_sec = cmd_timeout_sec

        self._active_mode: str = "idle"
        self._last_mode_change: float | None = None
        self._wheel_eligible_since: float | None = None
        self._last_cmd_vx: float = 0.0
        self._last_cmd_vy: float = 0.0
        self._last_cmd_wz: float = 0.0
        self._last_cmd_time: float | None = None

    # ── Input ─────────────────────────────────────────────────────────────────

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        self._last_cmd_vx = vx
        self._last_cmd_vy = vy
        self._last_cmd_wz = wz
        self._last_cmd_time = time.monotonic()

    # ── Tick → returns (vx, wz, wheel_mode) ──────────────────────────────────

    def tick(self) -> Tuple[float, float, bool]:
        """Return (vx, wz, wheel_mode) for LegController."""
        now = time.monotonic()
        requested, curv = self._requested_mode(now)
        selected = self._select_mode(requested, curv, now)

        if selected != self._active_mode:
            self._active_mode = selected
            self._last_mode_change = now

        if self._active_mode == "wheel":
            return self._last_cmd_vx, self._last_cmd_wz, True
        elif self._active_mode == "legged":
            # Pass through legged cmd (LegController handles it)
            return self._last_cmd_vx, self._last_cmd_wz, False
        else:  # idle
            return 0.0, 0.0, False

    # ── Internal mode logic (same as original) ────────────────────────────────

    def _is_recent(self, now: float) -> bool:
        return (self._last_cmd_time is not None and
                now - self._last_cmd_time <= self.cmd_timeout_sec)

    def _is_idle(self) -> bool:
        return (abs(self._last_cmd_vx) < self.thresholds.idle_linear and
                abs(self._last_cmd_vy) < self.thresholds.idle_lateral and
                abs(self._last_cmd_wz) < self.thresholds.idle_angular)

    def _requested_mode(self, now: float) -> Tuple[str, float]:
        if not self._is_recent(now) or self._is_idle():
            self._wheel_eligible_since = None
            return "idle", 0.0

        linear_x = abs(self._last_cmd_vx)
        linear_y = abs(self._last_cmd_vy)
        angular_z = abs(self._last_cmd_wz)
        curvature = angular_z / max(linear_x, 0.05)

        wheel_eligible = (
            self._last_cmd_vx > 0
            and linear_x >= self.thresholds.wheel_linear
            and linear_y <= self.thresholds.wheel_lateral
            and angular_z <= self.thresholds.wheel_angular
            and curvature <= self.thresholds.wheel_curvature
        )
        if wheel_eligible:
            if self._wheel_eligible_since is None:
                self._wheel_eligible_since = now
            if now - self._wheel_eligible_since >= self.wheel_engage_sustain_sec:
                return "wheel", curvature
        else:
            self._wheel_eligible_since = None

        return "legged", curvature

    def _select_mode(self, requested: str, curvature: float, now: float) -> str:
        if requested == self._active_mode:
            return requested
        # High-curvature emergency bypass: immediate switch wheel→legged for U-turn
        if (requested == "legged" and self._active_mode == "wheel" and
                curvature >= self.legged_override_curvature):
            return requested
        # Mode hold hysteresis
        if (self._active_mode in self.mode_hold_sec and
                self._last_mode_change is not None):
            held = now - self._last_mode_change
            if held < self.mode_hold_sec[self._active_mode]:
                return self._active_mode
        return requested

    @property
    def active_mode(self) -> str:
        return self._active_mode
