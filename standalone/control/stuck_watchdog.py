"""Stuck watchdog — ported from stuck_watchdog.py (rclpy / Nav2 action stripped).

Monitors robot displacement. When stuck (≤ threshold_m in window_sec while
a goal is active), emits a 'stuck_detected' event and commands a short reverse
cmd_vel burst directly (replaces the Nav2 BackUp action).

Recovery: publishes a Twist(-vx) to /<ns>/cmd_vel for backup_duration_sec,
then republishes the cached goal so the planner replans.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional, Tuple

from ..core.bus import BUS
from ..core.ros_compat import PoseStamped, String, Twist, now_stamp

Pose2D = Tuple[float, float, float]


class StuckWatchdog:
    """Outer-loop stuck recovery for the standalone exploration loop."""

    def __init__(
        self,
        namespace: str = "robot",
        *,
        stuck_window_sec: float = 10.0,
        stuck_threshold_m: float = 0.20,
        backup_distance_m: float = 0.40,
        backup_speed_mps: float = 0.10,
        cooldown_sec: float = 8.0,
        goal_change_threshold_m: float = 0.50,
    ) -> None:
        self._ns = namespace
        self._window = stuck_window_sec
        self._threshold = stuck_threshold_m
        self._backup_dist = backup_distance_m
        self._backup_speed = backup_speed_mps
        self._cooldown = cooldown_sec
        self._goal_change_thr = goal_change_threshold_m

        self._pose_hist: deque = deque()
        self._latest_goal: Optional[PoseStamped] = None
        self._last_recovery: float = 0.0
        self._recovery_in_flight: bool = False
        self._backup_start: Optional[float] = None
        self._backup_duration: float = backup_distance_m / max(backup_speed_mps, 0.01)

        odom_topic = f"/{namespace}/odom/nav"
        goal_topic = f"/{namespace}/goal_pose"
        nav_status_topic = f"/{namespace}/nav_status"
        self._cmd_vel_topic = f"/{namespace}/cmd_vel"
        self._recovery_topic = f"/{namespace}/recovery_event"
        self._goal_topic = goal_topic

        BUS.subscribe(odom_topic, self._on_odom)
        BUS.subscribe(goal_topic, self._on_goal)
        BUS.subscribe(nav_status_topic, self._on_nav_status)

    def _on_odom(self, msg) -> None:
        t = time.monotonic()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._pose_hist.append((t, x, y))
        cutoff = t - self._window
        while self._pose_hist and self._pose_hist[0][0] < cutoff:
            self._pose_hist.popleft()

    def _on_goal(self, msg: PoseStamped) -> None:
        prev = self._latest_goal
        if prev is not None:
            dx = msg.pose.position.x - prev.pose.position.x
            dy = msg.pose.position.y - prev.pose.position.y
            if math.hypot(dx, dy) > self._goal_change_thr:
                self._pose_hist.clear()
        self._latest_goal = msg

    def _on_nav_status(self, msg) -> None:
        """When the runtime declares the current goal unreachable, FORCE a
        backup recovery immediately (robot is probably wedged) and drop the
        cached goal so the post-backup republish doesn't fire on the
        known-bad target. The next planner cycle (VLM/CFPA2) will pick a
        fresh goal from the new pose after the robot has reversed clear."""
        try:
            import json
            payload = json.loads(getattr(msg, "data", "") or "{}")
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        if str(payload.get("state", "")) not in ("unreachable", "failed"):
            return
        # Drop the failed goal so post-backup republish (which uses
        # self._latest_goal) is a no-op — see tick() backup-done branch.
        self._latest_goal = None
        # Force backup recovery NOW so the robot escapes the wedged pose.
        # Respects cooldown to avoid recovery thrash if VLM keeps picking
        # adjacent unreachable goals in quick succession.
        t = time.monotonic()
        if t - self._last_recovery < self._cooldown:
            return
        if self._recovery_in_flight:
            return
        self._emit_recovery("stuck_detected")
        self._last_recovery = t
        self._recovery_in_flight = True
        self._backup_start = t
        self._emit_recovery("backup_started")

    # ── Called from main loop ─────────────────────────────────────────────────

    def tick(self) -> None:
        """Call this at ~2 Hz from the main sim loop."""
        t = time.monotonic()

        # Continue a backup in flight
        if self._recovery_in_flight:
            if self._backup_start is not None:
                elapsed = t - self._backup_start
                if elapsed < self._backup_duration:
                    cmd = Twist()
                    cmd.linear.x = -abs(self._backup_speed)
                    BUS.publish(self._cmd_vel_topic, cmd)
                    return
                else:
                    # Backup done — republish goal
                    self._emit_recovery("backup_done")
                    if self._latest_goal is not None:
                        BUS.publish(self._goal_topic, self._latest_goal)
                    self._pose_hist.clear()
                    self._recovery_in_flight = False
                    self._backup_start = None
            return

        if self._latest_goal is None:
            return
        if t - self._last_recovery < self._cooldown:
            return
        if not self._pose_hist:
            return

        oldest_t = self._pose_hist[0][0]
        latest_t = self._pose_hist[-1][0]
        if (latest_t - oldest_t) < self._window * 0.9:
            return

        xs = [p[1] for p in self._pose_hist]
        ys = [p[2] for p in self._pose_hist]
        moved = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        if moved >= self._threshold:
            return

        gx = self._latest_goal.pose.position.x
        gy = self._latest_goal.pose.position.y
        d2g = math.hypot(xs[-1] - gx, ys[-1] - gy)
        if d2g < 0.5:
            return

        self._emit_recovery("stuck_detected")
        self._last_recovery = t
        self._recovery_in_flight = True
        self._backup_start = t
        self._emit_recovery("backup_started")

    def _emit_recovery(self, kind: str) -> None:
        msg = String(data=kind)
        BUS.publish(self._recovery_topic, msg)
