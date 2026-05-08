#!/usr/bin/env python3
"""Live-MuJoCo wall/tip fail checker for headless FAR/RRT* runs.

Consumes `/mujoco/contacts` published by the patched mujoco_ros2_control
plugin — this is the *live* physics contact stream straight out of
`mjData.contact[]` after each `mj_step`, not a resynced copy. That means
wall hits the real sim actually experiences are always visible here,
regardless of odom drift or TF lag.

The checker fails on two independent conditions:

  1. Wall/divider contact:
     a physics contact where one geom name starts with a robot prefix
     (FL_/FR_/RL_/RR_/base_link/hip/thigh/calf/foot) AND the other name
     starts with `wall_` or `divider_`.

  2. Tip-over:
     robot body roll or pitch (read from /robot/odom/nav) exceeds 45°.

On either failure it prints a FAIL banner and exits with code 1, which
cascades through the launch's OnProcessExit handler and shuts the whole
nav_test_mujoco launch down.

Usage:
    python3 scripts/far_wall_checker.py
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


# Robot geom prefixes — any geom starting with these is a robot body part.
ROBOT_PREFIXES = ("FL_", "FR_", "RL_", "RR_", "base_collision",
                  "head_upper", "head_lower")

# Geoms that are SAFE to touch — ground plane and any explicitly allowed
# world objects. Everything else (walls, dividers, maze lanes, ramps,
# obstacles) is a fail if a robot geom touches it.
SAFE_GEOMS = {"ground", "green_marker_1", "green_marker_2",
              "green_marker_3", "box_obstacle_1", "box_obstacle_2"}

# Tip-over threshold (absolute roll or pitch).
TIPPED_OVER_RAD = math.radians(45.0)

# Status print interval (seconds).
STATUS_INTERVAL_SEC = 5.0

# Grace period at startup during which neither contact nor tip-over will
# be reported. Lets the robot finish initial settle / stand-up transient.
STARTUP_GRACE_SEC = 4.0


def _is_robot_geom(name: str) -> bool:
    return name.startswith(ROBOT_PREFIXES) or name == "_"

def _is_safe_geom(name: str) -> bool:
    return name in SAFE_GEOMS


def _roll_pitch_from_quat(q) -> tuple[float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class FarWallChecker(Node):
    def __init__(self) -> None:
        super().__init__("far_wall_checker")

        self._odom: Odometry | None = None
        self._last_contact_ncon = 0
        self._failed = False
        self._start_time = time.monotonic()

        contacts_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/mujoco/contacts", self._on_contacts, contacts_qos
        )
        self.create_subscription(
            Odometry, "/robot/odom/nav", self._on_odom, 10
        )
        self.create_timer(STATUS_INTERVAL_SEC, self._status_tick)

        print("=" * 64, flush=True)
        print(
            "  FAR WALL CHECKER — live /mujoco/contacts consumer\n"
            f"  Wall prefixes  : {WALL_PREFIXES}\n"
            f"  Allowed non-wall: {sorted(ALLOWED_NON_WALL_GEOMS)}\n"
            f"  Grace period   : {STARTUP_GRACE_SEC} s\n"
            f"  Tip threshold  : ±{math.degrees(TIPPED_OVER_RAD):.0f}°",
            flush=True,
        )
        print("=" * 64, flush=True)

    # ── Callbacks ─────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        if self._failed or self._odom is None:
            return
        if time.monotonic() - self._start_time < STARTUP_GRACE_SEC:
            return
        roll, pitch = _roll_pitch_from_quat(msg.pose.pose.orientation)
        if abs(roll) > TIPPED_OVER_RAD or abs(pitch) > TIPPED_OVER_RAD:
            self._fail_tip(roll, pitch)

    def _on_contacts(self, msg: String) -> None:
        if self._failed:
            return
        if time.monotonic() - self._start_time < STARTUP_GRACE_SEC:
            return

        lines = [ln for ln in msg.data.split("\n") if ln]
        self._last_contact_ncon = len(lines)

        for ln in lines:
            parts = ln.split("|")
            if len(parts) < 3:
                continue
            n1, n2, pos_str = parts[0], parts[1], parts[2]

            # Scene-agnostic check: is one side a robot geom and the
            # other side something that isn't safe (ground/markers)?
            # Works for any scene — walls, maze lanes, ramps, etc.
            r1 = _is_robot_geom(n1)
            r2 = _is_robot_geom(n2)
            if not (r1 or r2):
                # Neither side is a robot part — world-world contact, skip.
                continue

            robot = n1 if r1 else n2
            other = n2 if r1 else n1

            if _is_safe_geom(other):
                continue
            if _is_robot_geom(other):
                # Robot-robot self-contact (legs touching each other), skip.
                continue

            robot_label = robot if robot and robot != "_" else "<unnamed_robot_geom>"
            self._fail(robot_label, other, pos_str)
            return

    # ── Status ────────────────────────────────────────────────────────
    def _status_tick(self) -> None:
        if self._failed:
            return
        elapsed = time.monotonic() - self._start_time
        in_grace = elapsed < STARTUP_GRACE_SEC
        if self._odom is None:
            print(
                f"[wall_checker] t={elapsed:5.1f}s waiting for odom ncon={self._last_contact_ncon}",
                flush=True,
            )
            return
        p = self._odom.pose.pose.position
        roll, pitch = _roll_pitch_from_quat(self._odom.pose.pose.orientation)
        tag = "grace" if in_grace else "ok"
        print(
            f"[wall_checker] t={elapsed:5.1f}s {tag} "
            f"pose=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f}) "
            f"roll={math.degrees(roll):+5.1f}° pitch={math.degrees(pitch):+5.1f}° "
            f"ncon={self._last_contact_ncon}",
            flush=True,
        )

    # ── Failure paths ─────────────────────────────────────────────────
    def _fail(self, robot_geom: str, wall_geom: str, pos_str: str) -> None:
        self._failed = True
        elapsed = time.monotonic() - self._start_time
        p = self._odom.pose.pose.position if self._odom else None
        print("\n" + "!" * 64, flush=True)
        msg = (
            f"[wall_checker] FAIL after {elapsed:.1f}s — robot hit a wall\n"
            f"  robot geom : {robot_geom}\n"
            f"  wall  geom : {wall_geom}\n"
            f"  contact pt : {pos_str}"
        )
        if p is not None:
            msg += f"\n  robot pos  : ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f})"
        print(msg, flush=True)
        print("!" * 64, flush=True)
        self.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)

    def _fail_tip(self, roll: float, pitch: float) -> None:
        self._failed = True
        elapsed = time.monotonic() - self._start_time
        p = self._odom.pose.pose.position if self._odom else None
        print("\n" + "!" * 64, flush=True)
        msg = (
            f"[wall_checker] FAIL after {elapsed:.1f}s — robot tipped over\n"
            f"  roll       : {math.degrees(roll):+.1f}°\n"
            f"  pitch      : {math.degrees(pitch):+.1f}°\n"
            f"  threshold  : ±{math.degrees(TIPPED_OVER_RAD):.0f}°"
        )
        if p is not None:
            msg += f"\n  robot pos  : ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f})"
        print(msg, flush=True)
        print("!" * 64, flush=True)
        self.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)


def main() -> None:
    rclpy.init()
    node = FarWallChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
