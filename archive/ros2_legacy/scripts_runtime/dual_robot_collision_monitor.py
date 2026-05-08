#!/usr/bin/env python3
"""Dual-robot safety + diagnostic monitor for the demo3_mixed launch.

Designed to give an off-line reader (operator OR LLM agent post-mortem-ing
the terminal log) enough context to answer "why did the robot hit this wall
/ tip / get stuck?" without replaying the run. All inputs come from sources
the single-robot benchmark already trusts: `/mujoco/contacts` for physics,
`/{ns}/odom/ground_truth` for ground-truth pose, `/{ns}/map` for coverage.

Per-robot checks:
  1. WALL CONTACTS — /mujoco/contacts geom pairs classified by `b_` prefix
     into robot A vs B (walls = `wall_…` / `divider_…`).
  2. TIP-OVER — body-Z vs world-up tilt (gimbal-lock-free), latches when
     held above TIP_THRESHOLD_DEG for TIP_HOLD_SEC.
  3. PLANNER STUCK — goal active ≥ stuck_grace_sec AND distance-to-goal
     decreased < stuck_progress_m in last stuck_window_sec.
  4. COVERAGE — /{ns}/map free+occupied cells × cell area, ratio to
     scene_area_m2 (denominator matches session_reporter.py / benchmark).
  5. INTER-ROBOT CONTACTS — A↔B geom pairs (legacy).

Diagnostic context buffer:
  Every robot maintains a rolling 10 s history (sampled at 5 Hz) of
  (pose x/y/yaw, body-tilt, last v/ω commanded, latest /nav_status state,
  distance-to-goal). On every new wall / tip / stuck event the monitor
  WARN-prints the last 2 s of this buffer and the active goal so a reader
  can reconstruct what the planner + controller were doing in the moments
  leading up to the failure.

Output streams:
  - WARN per crash event (wall hit / tip / stuck) with snapshot block.
  - 10 s periodic INFO summary: per robot tilt, tip/stuck flags, wall hits,
    nav_state, v/ω, dist-to-goal, coverage %.
  - JSON report on shutdown: full per-robot dict + inter-robot pairs.

This script is launched by `nav_test_mujoco_fastlio_mixed.launch.py` and
appears on the `_NAV_DEBUG_KEEP_EXECUTABLES` keep-list, so its stdout
remains visible when the launch's `debug:=true` flag silences other nodes.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

# Shared contact classification + tilt math. Same module powers
# session_reporter.py so a "did the robot scuff anything?" verdict means
# the same thing in both places.
from contact_classify import (
    ROBOT_PART_PREFIXES, WALL_PREFIXES, GROUND_OR_HARMLESS_GEOMS,
    TIP_THRESHOLD_DEG, TIP_HOLD_SEC,
    TILT_DEGRADED_DEG, TILT_DEGRADED_HOLD_SEC,
    classify, roll_pitch_yaw_from_quat, tilt_from_quat_deg,
)

# Planner-stuck defaults — match the bar an operator would set by hand.
DEFAULT_STUCK_WINDOW_SEC = 15.0   # sliding window over which to measure progress
DEFAULT_STUCK_PROGRESS_M = 0.10   # < this much closer to goal in window → stuck
DEFAULT_STUCK_GRACE_SEC = 5.0     # ignore for first N s of every new goal

# Diagnostic state buffer (used by the context-snapshot dump on every
# crash event, so a reader can see pose/cmd/nav_state in the seconds
# leading up to the failure).
#
# Sample budget: an LLM agent reading the terminal post-mortem has limited
# context, so snapshots are deliberately small.
#   - STATE_SAMPLE_HZ × SNAPSHOT_DUMP_SEC = lines per crash event.
#   - 2.5 Hz × 2 s = 5 sample lines. Together with the 1-line crash banner,
#     each event costs ~6 lines — small enough that 10 events fit in well
#     under 100 lines, while still showing pre-impact velocity / heading
#     trend (5 evenly-spaced samples is enough to see "v stayed at 0.55
#     into the wall" or "tilt ramped 5°→90° in the last 0.4 s").
#   - STATE_BUFFER_SEC retained at 10 s so future longer-window dumps can
#     be added without code change. Buffer cost per robot: 25 samples ×
#     ~80 B = 2 KB. Negligible.
STATE_BUFFER_SEC = 10.0
STATE_SAMPLE_HZ = 2.5
SNAPSHOT_DUMP_SEC = 2.0


# classify(), tilt_from_quat_deg(), roll_pitch_yaw_from_quat() now live
# in contact_classify.py and are imported above. The helpers below remain
# local because they're cosmetic (formatting / 2D distance).

def _fmt(v: float | None) -> str:
    return f"{v:+.2f}" if v is not None else " n/a"


def msg_dist(a: tuple[float, float] | None, b: tuple[float, float] | None) -> float:
    if a is None or b is None:
        return float("nan")
    return math.hypot(a[0] - b[0], a[1] - b[1])


# Local aliases so existing call sites (`_tilt_from_quat_deg`,
# `_roll_pitch_yaw_from_quat`) continue to work without churn.
_tilt_from_quat_deg = tilt_from_quat_deg
_roll_pitch_yaw_from_quat = roll_pitch_yaw_from_quat


@dataclass
class StateSample:
    """One row of the rolling diagnostic buffer. Keep this small — many of
    these get printed to terminal on every crash event."""
    t_sec: float
    x: float
    y: float
    yaw: float
    tilt_deg: float
    v_cmd: float | None        # latest /{ns}/cmd_vel_stamped linear.x
    w_cmd: float | None        # latest /{ns}/cmd_vel_stamped angular.z
    nav_state: str | None      # latest /{ns}/nav_status `state` field
    dist_to_goal: float | None
    goal_xy: tuple[float, float] | None


class RobotChecks:
    """Per-robot rolling state (wall hits, tip-over, planner stuck, coverage)."""

    def __init__(self, ns: str, label: str,
                 stuck_window_sec: float, stuck_progress_m: float,
                 stuck_grace_sec: float, scene_area_m2: float):
        self.ns = ns
        self.label = label  # "A" or "B" — matches classify()

        # Contacts. We track two distinct streams:
        #   - wall_hits     : contacts with outer scene walls (wall_*, divider_*)
        #   - obstacle_hits : contacts with internal scene obstacles (any
        #                     other named non-ground non-self geom — boxes,
        #                     pillars, zigzags, dividers, ramps, …)
        # The session "did the robot scuff anything at all" verdict is the
        # `ever_touched_anything` boolean — true the first time EITHER
        # stream sees a contact, latched until shutdown. That's the simple
        # binary answer most checklists ("0 contacts pass criterion") want.
        self.wall_hits: list[dict] = []
        self.wall_hit_count_by_name: dict[str, int] = {}
        self.obstacle_hits: list[dict] = []
        self.obstacle_hit_count_by_name: dict[str, int] = {}
        self.robot_part_count_by_name: dict[str, int] = {}
        self.ever_touched_anything: bool = False
        self.first_touch_t_sec: float | None = None
        self.first_touch_kind: str | None = None      # "wall" or "obstacle"
        self.first_touch_geom: str | None = None      # the offending geom name

        # Pose / tip-over
        self.gt_received = False
        self.last_xy: tuple[float, float] | None = None
        self.last_yaw: float = 0.0
        self.peak_roll_deg = 0.0
        self.peak_pitch_deg = 0.0
        self.peak_tilt_deg = 0.0          # body-Z vs world-up (the canonical
                                          # tip metric — see _tilt_from_quat_deg).
        self.last_tilt_deg = 0.0
        self.tipped_over = False
        self.first_tip_t_sec: float | None = None
        self._tilt_above_since: float | None = None  # for TIP_HOLD_SEC gate

        # Degraded posture (leaned past gait-normal but not flipped).
        # Latches True after TILT_DEGRADED_HOLD_SEC sustained tilt above
        # TILT_DEGRADED_DEG; clears when tilt returns below threshold for
        # the same hold time (so a transient gait wobble doesn't trip it
        # and a momentary recovery doesn't false-clear it). The number
        # of distinct degraded periods is recorded — useful for "how
        # often does the robot need rescuing" benchmark.
        self.degraded_tilt = False
        self.degraded_tilt_count = 0          # distinct entries
        self.first_degraded_t_sec: float | None = None
        self._tilt_degraded_above_since: float | None = None
        self._tilt_degraded_below_since: float | None = None

        # Planner stuck
        self.stuck_window_sec = stuck_window_sec
        self.stuck_progress_m = stuck_progress_m
        self.stuck_grace_sec = stuck_grace_sec
        self.current_goal: tuple[float, float] | None = None
        self.goal_set_t_sec: float | None = None
        # rolling (t_sec, dist_to_goal) samples; trimmed to window.
        self.dist_history: deque[tuple[float, float]] = deque()
        self.last_dist_to_goal: float | None = None
        self.is_stuck = False
        self.stuck_events: list[dict] = []        # {t_sec, goal, dist, dt_window, progress_m}
        self.first_stuck_t_sec: float | None = None

        # Latest controller / planner status (cached, updated by callbacks).
        self.last_v_cmd: float | None = None
        self.last_w_cmd: float | None = None
        self.last_cmd_t: float | None = None
        self.last_nav_state: str | None = None
        self.last_nav_state_t: float | None = None

        # Coverage (from /{ns}/map). Numerator = (free + occupied) cells *
        # cell_area; denominator = scene_area_m2 (matches benchmark / session
        # reporter). Latest values are cached so JSON + summary line both see
        # them without iterating the OccupancyGrid again.
        self.scene_area_m2 = scene_area_m2
        self.map_received = False
        self.map_resolution_m = 0.0
        self.map_width_cells = 0
        self.map_height_cells = 0
        self.free_cells = 0
        self.occupied_cells = 0
        self.unknown_cells = 0
        self.explored_area_m2 = 0.0
        self.explored_fraction_of_grid = 0.0
        self.coverage_ratio_of_scene = 0.0   # 0..1, denominator = scene_area_m2

        # Diagnostic state buffer — sampled at STATE_SAMPLE_HZ, capped at
        # STATE_BUFFER_SEC. On every crash event, the last SNAPSHOT_DUMP_SEC
        # of this gets printed inline so a reader can reconstruct what the
        # planner / controller / pose were doing at the moment of failure.
        self.state_history: deque[StateSample] = deque()

    def on_contact(self, t_sec: float, kind: str, robot_geom: str,
                   other_geom: str, pos):
        """`kind` ∈ {'wall','obstacle'} — see classify(). Both feed the
        ever_touched_anything latch; per-stream lists keep them
        distinguishable for the JSON report."""
        rec = {
            "t_sec": round(t_sec, 3),
            "robot_geom": robot_geom,
            "other_geom": other_geom,
            "kind": kind,
            "pos": pos,
        }
        if kind == "wall":
            self.wall_hits.append(rec)
            self.wall_hit_count_by_name[other_geom] = \
                self.wall_hit_count_by_name.get(other_geom, 0) + 1
        else:
            self.obstacle_hits.append(rec)
            self.obstacle_hit_count_by_name[other_geom] = \
                self.obstacle_hit_count_by_name.get(other_geom, 0) + 1
        self.robot_part_count_by_name[robot_geom] = \
            self.robot_part_count_by_name.get(robot_geom, 0) + 1
        if not self.ever_touched_anything:
            self.ever_touched_anything = True
            self.first_touch_t_sec = t_sec
            self.first_touch_kind = kind
            self.first_touch_geom = other_geom

    def on_odom_gt(self, t_sec: float, x: float, y: float, yaw: float,
                   roll: float, pitch: float, tilt_deg: float):
        self.gt_received = True
        self.last_xy = (x, y)
        self.last_yaw = yaw
        rd, pd = math.degrees(roll), math.degrees(pitch)
        if abs(rd) > abs(self.peak_roll_deg):
            self.peak_roll_deg = rd
        if abs(pd) > abs(self.peak_pitch_deg):
            self.peak_pitch_deg = pd

        # Body-Z vs world-up — the actual tip metric. Latch only after the
        # tilt has held above threshold for TIP_HOLD_SEC (so a 1-frame
        # MuJoCo contact glitch doesn't latch a false tip).
        self.last_tilt_deg = tilt_deg
        if tilt_deg > self.peak_tilt_deg:
            self.peak_tilt_deg = tilt_deg
        if tilt_deg > TIP_THRESHOLD_DEG:
            if self._tilt_above_since is None:
                self._tilt_above_since = t_sec
            elif (t_sec - self._tilt_above_since) >= TIP_HOLD_SEC \
                    and not self.tipped_over:
                self.tipped_over = True
                self.first_tip_t_sec = t_sec
        else:
            self._tilt_above_since = None

        # Degraded posture latch — symmetric pattern but at the lower
        # threshold (30° / 5 s) and CAN clear if the robot recovers.
        # Caller (DualRobotSafetyMonitor) detects rising/falling edges
        # to print WARN/INFO events.
        if tilt_deg > TILT_DEGRADED_DEG:
            self._tilt_degraded_below_since = None
            if self._tilt_degraded_above_since is None:
                self._tilt_degraded_above_since = t_sec
            elif (t_sec - self._tilt_degraded_above_since) >= TILT_DEGRADED_HOLD_SEC \
                    and not self.degraded_tilt:
                self.degraded_tilt = True
                self.degraded_tilt_count += 1
                if self.first_degraded_t_sec is None:
                    self.first_degraded_t_sec = t_sec
        else:
            self._tilt_degraded_above_since = None
            if self.degraded_tilt:
                if self._tilt_degraded_below_since is None:
                    self._tilt_degraded_below_since = t_sec
                elif (t_sec - self._tilt_degraded_below_since) >= TILT_DEGRADED_HOLD_SEC:
                    self.degraded_tilt = False

        if self.current_goal is not None:
            d = math.hypot(self.current_goal[0] - x, self.current_goal[1] - y)
            self.last_dist_to_goal = d
            self.dist_history.append((t_sec, d))
            cutoff = t_sec - self.stuck_window_sec
            while self.dist_history and self.dist_history[0][0] < cutoff:
                self.dist_history.popleft()

    def on_goal(self, t_sec: float, gx: float, gy: float):
        # Re-arm on a *changed* goal. Otherwise CFPA2 republishing the same
        # waypoint at 1 Hz would constantly reset the stuck timer.
        if self.current_goal is not None:
            dxg = gx - self.current_goal[0]
            dyg = gy - self.current_goal[1]
            if math.hypot(dxg, dyg) < 0.20:
                return
        self.current_goal = (gx, gy)
        self.goal_set_t_sec = t_sec
        self.dist_history.clear()
        self.is_stuck = False  # re-armed for this leg

    def on_cmd_vel(self, t_sec: float, v: float, w: float) -> None:
        self.last_v_cmd = v
        self.last_w_cmd = w
        self.last_cmd_t = t_sec

    def on_nav_status(self, t_sec: float, state: str) -> None:
        self.last_nav_state = state
        self.last_nav_state_t = t_sec

    def on_map(self, msg: OccupancyGrid) -> None:
        free = occ = unk = 0
        for v in msg.data:
            if v < 0:
                unk += 1
            elif v >= 50:
                occ += 1
            else:
                free += 1
        res = float(msg.info.resolution)
        self.map_received = True
        self.map_resolution_m = res
        self.map_width_cells = int(msg.info.width)
        self.map_height_cells = int(msg.info.height)
        self.free_cells = free
        self.occupied_cells = occ
        self.unknown_cells = unk
        cell_area = res * res if res > 0 else 0.0
        known = free + occ
        self.explored_area_m2 = known * cell_area
        total = max(1, self.map_width_cells * self.map_height_cells)
        self.explored_fraction_of_grid = known / total
        if self.scene_area_m2 > 0:
            self.coverage_ratio_of_scene = self.explored_area_m2 / self.scene_area_m2

    def sample_state(self, t_sec: float) -> None:
        """Append the current state to the rolling history. Caller is the
        global STATE_SAMPLE_HZ timer in DualRobotSafetyMonitor; sampling
        outside that timer would distort the snapshot timestamps."""
        if self.last_xy is None:
            return
        self.state_history.append(StateSample(
            t_sec=t_sec,
            x=self.last_xy[0], y=self.last_xy[1], yaw=self.last_yaw,
            tilt_deg=self.last_tilt_deg,
            v_cmd=self.last_v_cmd, w_cmd=self.last_w_cmd,
            nav_state=self.last_nav_state,
            dist_to_goal=self.last_dist_to_goal,
            goal_xy=self.current_goal,
        ))
        cutoff = t_sec - STATE_BUFFER_SEC
        while self.state_history and self.state_history[0].t_sec < cutoff:
            self.state_history.popleft()

    def snapshot_lines(self, t_sec: float, window_sec: float) -> list[str]:
        """Compact 1-line-per-sample dump for the WARN block on each crash
        event. Format chosen to be terse for an LLM agent reading the log:
        units inferred from a single header line, fixed-precision columns,
        no per-sample re-stating of context. Five samples × ~70 chars each
        ≈ 350 chars per snapshot; the parent crash banner already carries
        the goal / part / wall / nav_state.
        """
        cutoff = t_sec - window_sec
        rows = [s for s in self.state_history if s.t_sec >= cutoff]
        if not rows:
            return ["  trail: <no samples in window>"]
        # One header (units), then one row per sample. yaw printed in
        # degrees (more readable than radians for the agent), tilt in
        # degrees, velocities signed. Truncated to last 5 evenly so the
        # block stays bounded even if STATE_SAMPLE_HZ is bumped.
        if len(rows) > 5:
            rows = rows[-5:]
        out = [
            "  trail t(s)  x      y     yaw°  tilt°    v     w     d2g  nav"
        ]
        for s in rows:
            v = f"{s.v_cmd:+5.2f}" if s.v_cmd is not None else "  n/a"
            w = f"{s.w_cmd:+5.2f}" if s.w_cmd is not None else "  n/a"
            d = f"{s.dist_to_goal:5.2f}" if s.dist_to_goal is not None else "  n/a"
            ns = (s.nav_state or "?")[:18]
            out.append(
                f"        {s.t_sec:5.2f} {s.x:+5.2f} {s.y:+5.2f} "
                f"{math.degrees(s.yaw):+6.1f} {s.tilt_deg:5.1f} "
                f"{v} {w} {d} {ns}"
            )
        return out

    def evaluate_stuck(self, t_sec: float) -> bool:
        """Update is_stuck. Returns True iff a NEW stuck event was logged."""
        if self.current_goal is None or self.goal_set_t_sec is None:
            return False
        if (t_sec - self.goal_set_t_sec) < self.stuck_grace_sec:
            return False
        # Need a window of samples to compare.
        if len(self.dist_history) < 2:
            return False
        oldest_t, oldest_d = self.dist_history[0]
        if (t_sec - oldest_t) < self.stuck_window_sec * 0.8:
            # Window not yet full — don't false-fire.
            return False
        newest_d = self.dist_history[-1][1]
        progress = oldest_d - newest_d  # positive = closer
        if progress < self.stuck_progress_m:
            ev = {
                "t_sec": round(t_sec, 3),
                "goal": [round(self.current_goal[0], 3), round(self.current_goal[1], 3)],
                "dist_to_goal_m": round(newest_d, 3),
                "window_sec": round(t_sec - oldest_t, 2),
                "progress_m": round(progress, 4),
            }
            self.stuck_events.append(ev)
            if not self.is_stuck:
                self.is_stuck = True
                self.first_stuck_t_sec = t_sec
                return True
        else:
            # Progress recovered → un-flag, but keep the historical event log.
            self.is_stuck = False
        return False

    def to_dict(self) -> dict:
        return {
            "namespace": self.ns,
            # ── single-bit "did the robot scuff ANYTHING this session?" ──
            # Latched true on first wall OR obstacle contact, never resets.
            # This is the field most pass/fail checklists want:
            #   ever_touched=True  → fail the safety criterion, regardless
            #                        of whether it self-recovered.
            #   ever_touched=False → clean run.
            # The detailed per-stream lists below give context for *what*
            # was scuffed when ever_touched is True.
            "ever_touched": {
                "any": self.ever_touched_anything,
                "first_t_sec": self.first_touch_t_sec,
                "first_kind": self.first_touch_kind,
                "first_geom": self.first_touch_geom,
                "wall_hits_total": len(self.wall_hits),
                "obstacle_hits_total": len(self.obstacle_hits),
            },
            "wall_contacts": {
                "count": len(self.wall_hits),
                "first_t_sec": self.wall_hits[0]["t_sec"] if self.wall_hits else None,
                "by_wall": dict(sorted(self.wall_hit_count_by_name.items(),
                                       key=lambda kv: -kv[1])),
                "by_robot_part": dict(sorted(self.robot_part_count_by_name.items(),
                                             key=lambda kv: -kv[1])),
                "events": self.wall_hits[:50],
                "events_truncated": len(self.wall_hits) > 50,
            },
            "obstacle_contacts": {
                "count": len(self.obstacle_hits),
                "first_t_sec": (self.obstacle_hits[0]["t_sec"]
                                if self.obstacle_hits else None),
                "by_obstacle": dict(sorted(
                    self.obstacle_hit_count_by_name.items(),
                    key=lambda kv: -kv[1])),
                "events": self.obstacle_hits[:50],
                "events_truncated": len(self.obstacle_hits) > 50,
            },
            "tipped_over": {
                "tripped": self.tipped_over,
                "first_t_sec": self.first_tip_t_sec,
                # Canonical tip metric: angle between body-Z and world-up.
                # Trip rule = peak_tilt_deg > threshold AND held ≥ hold_sec.
                "peak_tilt_deg": round(self.peak_tilt_deg, 2),
                "last_tilt_deg": round(self.last_tilt_deg, 2),
                "tilt_threshold_deg": TIP_THRESHOLD_DEG,
                "tilt_hold_sec": TIP_HOLD_SEC,
                # Kept for human reading; not used by the trip rule.
                "peak_roll_deg": round(self.peak_roll_deg, 2),
                "peak_pitch_deg": round(self.peak_pitch_deg, 2),
            },
            "degraded_tilt": {
                # Sub-tip "leaned past gait-normal" detector. Currently
                # latched True iff tilt has held > TILT_DEGRADED_DEG for
                # ≥ TILT_DEGRADED_HOLD_SEC AND has not yet recovered for
                # the same hold time. `count` is distinct entries — a
                # robot that leans, recovers, leans again counts twice.
                "currently_degraded": self.degraded_tilt,
                "first_t_sec": self.first_degraded_t_sec,
                "entry_count": self.degraded_tilt_count,
                "tilt_threshold_deg": TILT_DEGRADED_DEG,
                "tilt_hold_sec": TILT_DEGRADED_HOLD_SEC,
            },
            "planner_stuck": {
                "currently_stuck": self.is_stuck,
                "first_t_sec": self.first_stuck_t_sec,
                "event_count": len(self.stuck_events),
                "current_goal": list(self.current_goal) if self.current_goal else None,
                "last_dist_to_goal_m": (
                    round(self.last_dist_to_goal, 3)
                    if self.last_dist_to_goal is not None else None
                ),
                "window_sec": self.stuck_window_sec,
                "progress_threshold_m": self.stuck_progress_m,
                "grace_sec": self.stuck_grace_sec,
                "events": self.stuck_events[:30],
                "events_truncated": len(self.stuck_events) > 30,
            },
            "coverage": {
                "map_received": self.map_received,
                "resolution_m": round(self.map_resolution_m, 3),
                "width_cells": self.map_width_cells,
                "height_cells": self.map_height_cells,
                "free_cells": self.free_cells,
                "occupied_cells": self.occupied_cells,
                "unknown_cells": self.unknown_cells,
                "explored_area_m2": round(self.explored_area_m2, 3),
                "explored_fraction_of_grid": round(self.explored_fraction_of_grid, 4),
                "scene_area_m2": self.scene_area_m2,
                "coverage_ratio_of_scene": round(self.coverage_ratio_of_scene, 4),
                "coverage_pass_90pct": (
                    self.scene_area_m2 > 0 and self.coverage_ratio_of_scene >= 0.90
                ),
            },
            "controller": {
                "last_v_cmd": (
                    round(self.last_v_cmd, 3)
                    if self.last_v_cmd is not None else None
                ),
                "last_w_cmd": (
                    round(self.last_w_cmd, 3)
                    if self.last_w_cmd is not None else None
                ),
                "last_cmd_t_sec": (
                    round(self.last_cmd_t, 3)
                    if self.last_cmd_t is not None else None
                ),
                "last_nav_state": self.last_nav_state,
                "last_nav_state_t_sec": (
                    round(self.last_nav_state_t, 3)
                    if self.last_nav_state_t is not None else None
                ),
            },
        }


class DualRobotSafetyMonitor(Node):
    def __init__(
        self,
        output_path: str | None,
        verbose: bool,
        robots: list[str],
        stuck_window_sec: float,
        stuck_progress_m: float,
        stuck_grace_sec: float,
        scene_area_m2: float,
    ):
        super().__init__("dual_robot_collision_monitor")
        self.output_path = output_path
        self.verbose = verbose
        self.t0 = time.time()

        # Map "A"/"B" classify() → namespace. demo3_mixed has robot_a (bare
        # geoms) and robot_b (b_-prefixed). If a future scene reverses this,
        # pass --robots in matching order.
        if len(robots) != 2:
            raise SystemExit("--robots must be exactly two namespaces (A then B)")
        self.ns_a, self.ns_b = robots[0], robots[1]
        self.scene_area_m2 = scene_area_m2
        self.checks: dict[str, RobotChecks] = {
            "A": RobotChecks(self.ns_a, "A", stuck_window_sec,
                             stuck_progress_m, stuck_grace_sec, scene_area_m2),
            "B": RobotChecks(self.ns_b, "B", stuck_window_sec,
                             stuck_progress_m, stuck_grace_sec, scene_area_m2),
        }

        # Inter-robot pair tracking (kept from prior version).
        self.inter_robot_pairs: dict[tuple[str, str], dict] = {}
        self.contact_msgs = 0
        self.inter_robot_hits_total = 0
        self.last_log_t = 0.0

        # /mujoco/contacts is BEST_EFFORT — match QoS or DDS silently drops it.
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=50,
        )
        # Octomap publishes /{ns}/map RELIABLE+TRANSIENT_LOCAL — match it.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(String, "/mujoco/contacts",
                                 self._on_contacts, be_qos)

        # Per-robot subscriptions. Each is small and rclpy callbacks run on
        # the same executor, so this is single-threaded — no locking needed.
        for label, ns in (("A", self.ns_a), ("B", self.ns_b)):
            self.create_subscription(
                Odometry, f"/{ns}/odom/ground_truth",
                lambda msg, lab=label: self._on_odom_gt(lab, msg), 10)
            # CFPA2 / FAR / VLM publish goals on /{ns}/way_point_coord.
            self.create_subscription(
                PointStamped, f"/{ns}/way_point_coord",
                lambda msg, lab=label: self._on_goal(lab, msg), 10)
            # Controller stream — astar_nav / pathFollower publish here.
            # twist_bridge re-emits as plain Twist on /{ns}/cmd_vel; we
            # listen to the stamped version because that's what the planner
            # itself emits, so latency is zero (no bridge hop).
            self.create_subscription(
                TwistStamped, f"/{ns}/cmd_vel_stamped",
                lambda msg, lab=label: self._on_cmd_vel(lab, msg), 10)
            # Nav state JSON string from astar_nav / far_status_adapter.
            self.create_subscription(
                String, f"/{ns}/nav_status",
                lambda msg, lab=label: self._on_nav_status(lab, msg), 10)
            # OccupancyGrid for coverage. Same QoS as octomap publisher.
            self.create_subscription(
                OccupancyGrid, f"/{ns}/map",
                lambda msg, lab=label: self._on_map(lab, msg), map_qos)

        # Three timers:
        #   0.5  s — cheap stuck evaluation against the rolling progress buffer
        #   1.0/STATE_SAMPLE_HZ — append a StateSample to each robot's history
        #   10.0 s — human-readable summary line per robot
        self.create_timer(0.5, self._tick_stuck)
        self.create_timer(1.0 / STATE_SAMPLE_HZ, self._sample_states)
        self.create_timer(10.0, self._periodic_summary)
        self.get_logger().info(
            f"dual_robot_safety_monitor started (A={self.ns_a}, B={self.ns_b}); "
            f"scene_area={scene_area_m2:.1f} m² (90% bar = "
            f"{0.9 * scene_area_m2:.1f} m²); "
            f"output={self.output_path or '<stdout only>'}"
        )

    # ── Callbacks ────────────────────────────────────────────────────
    def _now(self) -> float:
        return time.time() - self.t0

    def _on_contacts(self, msg: String):
        self.contact_msgs += 1
        now = self._now()
        for raw in msg.data.split("\n"):
            line = raw.strip()
            if not line or "|" not in line:
                continue
            parts = line.split("|")
            if len(parts) < 3:
                continue
            g1, g2, pos_str = parts[0], parts[1], parts[2]
            try:
                xyz = [float(x) for x in pos_str.split(",")]
            except ValueError:
                xyz = [float("nan")] * 3
            c1, c2 = classify(g1), classify(g2)

            # Inter-robot
            if (c1 == "A" and c2 == "B") or (c1 == "B" and c2 == "A"):
                key = (g1, g2) if c1 == "A" else (g2, g1)
                self.inter_robot_hits_total += 1
                if key not in self.inter_robot_pairs:
                    self.inter_robot_pairs[key] = {
                        "hits": 1, "first_sec": now, "first_pos": xyz,
                    }
                    if self.verbose:
                        self.get_logger().warn(
                            f"INTER-ROBOT CONTACT @ t={now:.1f}s: "
                            f"A.{key[0]} × B.{key[1]} @ {xyz}"
                        )
                else:
                    self.inter_robot_pairs[key]["hits"] += 1
                continue

            # Per-robot scuff. We accept either side being a wall OR an
            # interior obstacle; ground / robot-self contacts are dropped.
            # The `kind` field flows through to the JSON for context.
            other_kind = None
            if   c1 in ("wall", "obstacle") and c2 == "A": who, other, kind, robot_geom = "a", g1, c1, g2
            elif c1 in ("wall", "obstacle") and c2 == "B": who, other, kind, robot_geom = "b", g1, c1, g2
            elif c2 in ("wall", "obstacle") and c1 == "A": who, other, kind, robot_geom = "a", g2, c2, g1
            elif c2 in ("wall", "obstacle") and c1 == "B": who, other, kind, robot_geom = "b", g2, c2, g1
            else:
                continue
            label = "A" if who == "a" else "B"
            c = self.checks[label]
            c.on_contact(now, kind, robot_geom, other, xyz)
            # Always announce — without this in the terminal, an agent
            # reading the log has no signal that a scuff happened at all.
            self._announce_scuff(now, label, c, kind, robot_geom, other, xyz)

    def _announce_scuff(self, now, label, c, kind, robot_geom, other, xyz):
        # Throttle the snapshot dump to 1 per 2 s per robot — a single
        # contact often produces a burst of ~5–20 contacts as the robot
        # presses against the wall, and dumping a 2 s table on each one
        # would drown the terminal. The first hit gets the full block; the
        # next ones in the burst just get a one-liner counter.
        if not hasattr(self, "_last_scuff_dump_t"):
            self._last_scuff_dump_t = {"A": -1e9, "B": -1e9}
        # Always publish a structured event on /collision_events so
        # downstream tools (failure_decomposer, dataset loggers) get
        # one JSON per real contact, regardless of the announce throttle.
        if not hasattr(self, "_collision_event_pub"):
            self._collision_event_pub = self.create_publisher(
                String, "/collision_events", 50)
        try:
            self._collision_event_pub.publish(String(
                data=json.dumps({
                    "schema": "collision_event/v1",
                    "t_sim": round(now, 3),
                    "robot": c.ns,
                    "kind": kind,
                    "part": robot_geom,
                    "other": other,
                    "pos": [round(xyz[0], 3), round(xyz[1], 3), round(xyz[2], 3)],
                    "goal": (list(c.current_goal) if c.current_goal else None),
                    "nav_state": c.last_nav_state,
                    "v_cmd": c.last_v_cmd,
                    "w_cmd": c.last_w_cmd,
                    "tilt_deg": c.last_tilt_deg,
                    "ever_touched": bool(c.ever_touched_anything),
                })))
        except Exception:
            # If the publisher hiccups don't take down the monitor.
            pass
        if (now - self._last_scuff_dump_t[label]) > 2.0:
            self._last_scuff_dump_t[label] = now
            goal_str = (
                f"({c.current_goal[0]:.2f},{c.current_goal[1]:.2f})"
                if c.current_goal else "<no goal>"
            )
            d2g = (f"{c.last_dist_to_goal:.2f}m"
                   if c.last_dist_to_goal is not None else "n/a")
            tag = "WALL CONTACT" if kind == "wall" else "OBSTACLE SCUFF"
            self.get_logger().warn(
                f"{tag} @ t={now:.2f}s: {label}={c.ns} "
                f"part={robot_geom} × {other} pos=[{xyz[0]:+.2f},"
                f"{xyz[1]:+.2f},{xyz[2]:+.2f}] | goal={goal_str} "
                f"d2g={d2g} nav={c.last_nav_state or '?'} "
                f"v={_fmt(c.last_v_cmd)} w={_fmt(c.last_w_cmd)} "
                f"tilt={c.last_tilt_deg:.1f}° "
                f"walls={len(c.wall_hits)} obstacles={len(c.obstacle_hits)} "
                f"ever_touched={'Y' if c.ever_touched_anything else 'N'}"
            )
            for line in c.snapshot_lines(now, SNAPSHOT_DUMP_SEC):
                self.get_logger().warn(line)

    def _on_odom_gt(self, label: str, msg: Odometry):
        now = self._now()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _roll_pitch_yaw_from_quat(q)
        tilt_deg = _tilt_from_quat_deg(q)
        c = self.checks[label]
        was_tipped = c.tipped_over
        was_degraded = c.degraded_tilt
        c.on_odom_gt(now, float(p.x), float(p.y), yaw, roll, pitch, tilt_deg)

        if c.tipped_over and not was_tipped:
            goal_str = (
                f"({c.current_goal[0]:.2f},{c.current_goal[1]:.2f})"
                if c.current_goal else "<no goal>"
            )
            self.get_logger().warn(
                f"TIP-OVER @ t={now:.2f}s: {label}={c.ns} "
                f"tilt={tilt_deg:.1f}° (held >{TIP_THRESHOLD_DEG:.0f}° for "
                f"{TIP_HOLD_SEC:.1f}s) roll={math.degrees(roll):+.1f}° "
                f"pitch={math.degrees(pitch):+.1f}° | "
                f"pos=({p.x:+.2f},{p.y:+.2f}) yaw={yaw:+.2f} | "
                f"goal={goal_str} nav={c.last_nav_state or '?'} "
                f"v={_fmt(c.last_v_cmd)} w={_fmt(c.last_w_cmd)}"
            )
            for line in c.snapshot_lines(now, SNAPSHOT_DUMP_SEC):
                self.get_logger().warn(line)

        # Degraded-tilt edges — warn on entering, info on recovering.
        # Skipped if a full TIP-OVER happened (the bigger event subsumes).
        if c.degraded_tilt and not was_degraded and not c.tipped_over:
            goal_str = (
                f"({c.current_goal[0]:.2f},{c.current_goal[1]:.2f})"
                if c.current_goal else "<no goal>"
            )
            self.get_logger().warn(
                f"DEGRADED-TILT @ t={now:.2f}s: {label}={c.ns} "
                f"tilt={tilt_deg:.1f}° (held >{TILT_DEGRADED_DEG:.0f}° for "
                f"{TILT_DEGRADED_HOLD_SEC:.1f}s) — robot leaned far past "
                f"gait-normal but not flipped. Likely a body part hung up "
                f"on something; planner can't recover, external "
                f"intervention may be needed. "
                f"pos=({p.x:+.2f},{p.y:+.2f}) yaw={yaw:+.2f} "
                f"goal={goal_str} nav={c.last_nav_state or '?'} "
                f"v={_fmt(c.last_v_cmd)} w={_fmt(c.last_w_cmd)} "
                f"degraded_count={c.degraded_tilt_count}"
            )
            for line in c.snapshot_lines(now, SNAPSHOT_DUMP_SEC):
                self.get_logger().warn(line)
        elif was_degraded and not c.degraded_tilt and not c.tipped_over:
            self.get_logger().info(
                f"DEGRADED-TILT cleared @ t={now:.2f}s: {label}={c.ns} "
                f"tilt={tilt_deg:.1f}° back below {TILT_DEGRADED_DEG:.0f}° "
                f"for {TILT_DEGRADED_HOLD_SEC:.1f}s. "
                f"pos=({p.x:+.2f},{p.y:+.2f})"
            )

    def _on_goal(self, label: str, msg: PointStamped):
        now = self._now()
        c = self.checks[label]
        prev = c.current_goal
        c.on_goal(now, float(msg.point.x), float(msg.point.y))
        # Only announce truly-new goals (on_goal already filtered <0.20 m
        # republishes by leaving current_goal unchanged).
        if c.current_goal != prev:
            self.get_logger().info(
                f"NEW GOAL  @ t={now:.2f}s: {label}={c.ns} "
                f"goal=({c.current_goal[0]:.2f},{c.current_goal[1]:.2f}) "
                f"from pose=({c.last_xy[0]:+.2f},{c.last_xy[1]:+.2f}) "
                f"d={msg_dist(c.last_xy, c.current_goal):.2f}m" if c.last_xy
                else f"NEW GOAL  @ t={now:.2f}s: {label}={c.ns} "
                f"goal=({c.current_goal[0]:.2f},{c.current_goal[1]:.2f})"
            )

    def _on_cmd_vel(self, label: str, msg: TwistStamped):
        now = self._now()
        self.checks[label].on_cmd_vel(
            now, float(msg.twist.linear.x), float(msg.twist.angular.z))

    def _on_nav_status(self, label: str, msg: String):
        now = self._now()
        # nav_status payload is a JSON string from astar_nav / far_status
        # adapter (`{"schema":"nav_status/v1","source":...,"state":...,...}`).
        # Pull just the `state` field for compactness.
        state = msg.data
        try:
            j = json.loads(msg.data)
            if isinstance(j, dict) and "state" in j:
                state = str(j["state"])
        except (ValueError, TypeError):
            pass
        # Crop to keep log lines bounded.
        self.checks[label].on_nav_status(now, state[:48])

    def _on_map(self, label: str, msg: OccupancyGrid):
        self.checks[label].on_map(msg)

    # ── Timers ───────────────────────────────────────────────────────
    def _tick_stuck(self):
        now = self._now()
        for label, check in self.checks.items():
            newly_stuck = check.evaluate_stuck(now)
            if newly_stuck:
                ev = check.stuck_events[-1]
                self.get_logger().warn(
                    f"PLANNER STUCK @ t={now:.2f}s: {label}={check.ns} "
                    f"goal={ev['goal']} dist={ev['dist_to_goal_m']:.2f}m "
                    f"progress={ev['progress_m']:+.3f}m over "
                    f"{ev['window_sec']:.1f}s | "
                    f"pose=({check.last_xy[0]:+.2f},{check.last_xy[1]:+.2f}) "
                    f"yaw={check.last_yaw:+.2f} nav={check.last_nav_state or '?'} "
                    f"v={_fmt(check.last_v_cmd)} w={_fmt(check.last_w_cmd)}"
                )
                for line in check.snapshot_lines(now, SNAPSHOT_DUMP_SEC):
                    self.get_logger().warn(line)

    def _sample_states(self):
        """5 Hz — append a StateSample to each robot's rolling history."""
        now = self._now()
        for c in self.checks.values():
            c.sample_state(now)

    def _periodic_summary(self):
        now = self._now()
        if now - self.last_log_t < 9.5:
            return
        self.last_log_t = now

        # One dense line per robot per cycle. Format:
        # [t=12.5s] A=robot_a (+5.2,+2.9)/+1.32 v+0.55 w-0.08 nav=... goal=(+8.2,+4.0) d2g=3.55 walls=0 tilt=2.1°(pk4.5) tip=N stuck=N cov=23.4%
        # Twenty seconds of monitoring = 4 lines. Field order is consistent
        # so a regex / agent can pick out e.g. tilt or coverage trivially.
        def _line(label: str) -> str:
            c = self.checks[label]
            xy = (
                f"({c.last_xy[0]:+.2f},{c.last_xy[1]:+.2f})"
                if c.last_xy else "<no_odom>"
            )
            goal = (
                f"({c.current_goal[0]:+.2f},{c.current_goal[1]:+.2f})"
                if c.current_goal else "<no_goal>"
            )
            d2g = (f"{c.last_dist_to_goal:5.2f}"
                   if c.last_dist_to_goal is not None else "  n/a")
            cov = (
                f"{c.coverage_ratio_of_scene*100:5.1f}%"
                if (c.map_received and self.scene_area_m2 > 0)
                else ("nomap" if not c.map_received else "noden")
            )
            nav = (c.last_nav_state or "?")[:14]
            # `touched=Y/N` is the latched ever_touched_anything bit;
            # `c=` is wall+obstacle contact total for this session.
            total_c = len(c.wall_hits) + len(c.obstacle_hits)
            # `degr` is the new degraded-tilt latch (Y/N + entry count).
            # Distinct from `tip` which only fires past 70°/1s.
            degr = (f"Y/{c.degraded_tilt_count}"
                    if c.degraded_tilt
                    else (f"N/{c.degraded_tilt_count}"
                          if c.degraded_tilt_count > 0 else "N"))
            return (
                f"{label}={c.ns} {xy}/{c.last_yaw:+.2f} "
                f"v{_fmt(c.last_v_cmd)} w{_fmt(c.last_w_cmd)} "
                f"nav={nav:<14} goal={goal} d2g={d2g} "
                f"touched={'Y' if c.ever_touched_anything else 'N'} "
                f"c={total_c:>2d} "
                f"tilt={c.last_tilt_deg:4.1f}°(pk{c.peak_tilt_deg:4.1f}) "
                f"tip={'Y' if c.tipped_over else 'N'} "
                f"degr={degr} "
                f"stuck={'Y' if c.is_stuck else 'N'} cov={cov}"
            )

        self.get_logger().info(f"[t={now:6.1f}s] {_line('A')}")
        self.get_logger().info(f"[t={now:6.1f}s] {_line('B')}")

    # ── Report ───────────────────────────────────────────────────────
    def write_report(self):
        report = {
            "elapsed_sec": time.time() - self.t0,
            "contact_msgs_received": self.contact_msgs,
            "robots": {
                self.ns_a: self.checks["A"].to_dict(),
                self.ns_b: self.checks["B"].to_dict(),
            },
            "inter_robot": {
                "hits_total": self.inter_robot_hits_total,
                "unique_pairs": len(self.inter_robot_pairs),
                "pairs": [
                    {
                        "robot_a_geom": k[0], "robot_b_geom": k[1],
                        "hits": v["hits"],
                        "first_sec": round(v["first_sec"], 3),
                        "first_pos": v["first_pos"],
                    }
                    for k, v in sorted(self.inter_robot_pairs.items(),
                                       key=lambda kv: -kv[1]["hits"])
                ],
            },
        }
        if self.output_path:
            Path(self.output_path).parent.mkdir(parents=True, exist_ok=True)
            Path(self.output_path).write_text(json.dumps(report, indent=2))
            self.get_logger().info(f"wrote report to {self.output_path}")
        else:
            print("\n=== DUAL-ROBOT SAFETY REPORT ===")
            print(json.dumps(report, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=os.environ.get(
            "DUAL_COLLISION_OUTPUT",
            "/tmp/dual_robot_collision_report.json",
        ),
        help="Path to write final JSON report (default: $DUAL_COLLISION_OUTPUT "
        "or /tmp/dual_robot_collision_report.json).",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Log each newly-seen wall hit and inter-robot geom pair "
             "(default: 10s summary + WARN on stuck/tip events only).",
    )
    parser.add_argument(
        "--robots", nargs=2, default=["robot_a", "robot_b"],
        metavar=("NS_A", "NS_B"),
        help="Two namespaces in order (A then B). Default: robot_a robot_b.",
    )
    parser.add_argument("--stuck-window-sec", type=float,
                        default=DEFAULT_STUCK_WINDOW_SEC,
                        help="Sliding window over which planner-stuck progress "
                             "is measured.")
    parser.add_argument("--stuck-progress-m", type=float,
                        default=DEFAULT_STUCK_PROGRESS_M,
                        help="Distance-to-goal must shrink by at least this "
                             "much within the window or robot is stuck.")
    parser.add_argument("--stuck-grace-sec", type=float,
                        default=DEFAULT_STUCK_GRACE_SEC,
                        help="Ignore stuck check for this long after every new "
                             "goal — gives the planner time to spin up.")
    parser.add_argument("--scene-area-m2", type=float, default=384.0,
                        help="Sim ground-truth observable area, used as the "
                             "denominator for coverage_ratio_of_scene. "
                             "Default 384.0 (demo3_mixed 24×16 m). Pass 0 to "
                             "disable the coverage % column.")

    if argv is None:
        argv = sys.argv[1:]
    ros_args = []
    if "--ros-args" in argv:
        idx = argv.index("--ros-args")
        argv, ros_args = argv[:idx], argv[idx:]
    args = parser.parse_args(argv)

    rclpy.init(args=ros_args or None)
    node = DualRobotSafetyMonitor(
        output_path=args.output,
        verbose=args.verbose,
        robots=args.robots,
        stuck_window_sec=args.stuck_window_sec,
        stuck_progress_m=args.stuck_progress_m,
        stuck_grace_sec=args.stuck_grace_sec,
        scene_area_m2=args.scene_area_m2,
    )

    def _shutdown(*_):
        node.write_report()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.write_report()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
