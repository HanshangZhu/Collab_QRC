#!/usr/bin/env python3
"""Velocity-aware safety supervisor for the CMU local-planner output.

Sits between `pathFollower` and `twist_bridge` on the cmd_vel path. Reads the
LiDAR 2D scan and the commanded velocity; caps the commanded velocity so that
the robot's stopping distance always stays within the nearest forward
obstacle clearance. Lets the robot cruise fast in open space and slow down
automatically near walls.

Model
-----
For a commanded speed `v_cmd`, a maximum deceleration `a_max`, and a nearest
obstacle in the direction of motion at range `d_nearest`, the supervisor
computes:

    d_safe    = safety_margin_m  # hard "never get closer than" buffer
    d_reserve = max(0, d_nearest - d_safe)
    v_cap     = sqrt(2 * a_max * d_reserve)

and then outputs

    v_out = min(|v_cmd|, v_cap, max_linear_speed) · sign(v_cmd.x)

preserving the sign of the forward component (so reverse primitives from
`twoWayDrive=true` still work — the supervisor just limits magnitude).
Angular velocity is scaled by the same clamp ratio so instantaneous curvature
is preserved.

Topics
------
* Sub  `<scan_topic>` (LaserScan)        — nearest-obstacle source. 2D scan.
* Sub  `<cmd_in_topic>` (TwistStamped)   — commanded velocity from pathFollower.
* Pub  `<cmd_out_topic>` (TwistStamped)  — capped velocity → twist_bridge.

Parameters (ROS2)
-----------------
max_linear_speed_m_s   default 0.4    absolute ceiling (matches far_max_speed)
max_decel_m_s2         default 2.0    matches localPlanner `maxAccel`
safety_margin_m        default 0.10   matches localPlanner `stopDisThre`
forward_arc_half_rad   default 1.047  ±60° forward cone for nearest-obstacle
                                      search — sideways/rear obstacles don't
                                      gate forward motion
min_valid_range_m      default 0.05   scan readings below this are discarded
stale_cmd_timeout_sec  default 0.5    if no cmd_vel arrives for this long,
                                      publish a zero twist (hard stop)
publish_rate_hz        default 50     matches pathFollower cadence
debug                  default false  emit extra log lines for diagnostics

History
-------
This is the "cone v1" version. A body-aware rewrite (corridor check using
`|x| < L/2 ∧ |y| < W/2 + buffer`) was tried on 2026-04-15 to address the
rear-scrape failure observed at speed=0.4 with the cone version. The
body-aware version regressed (self-return false positives on the robot's
own legs in the first pass; a tip-over on the second pass with the
self-filter) and was reverted. See the Phase 5 tuning notes in CLAUDE.md
for the full iteration history. If someone revisits this, start from the
body-aware v2 state with these lessons baked in: (a) scan points within
the physical body rectangle are self-returns and must be filtered, (b)
an "abreast-threat" hard-stop is too aggressive because it interacts
badly with pathFollower's velocity state machine, and (c) the LiDAR's
forward mount offset (`0.16 m` on Go2W) shifts the body-frame coordinate
frame relative to the scan frame.

Integration
-----------
This node is NOT wired into `nav_test_mujoco.launch.py` by default; enable
via `enable_velocity_supervisor:=true` launch arg. When enabled:

  1. pathFollower's `/cmd_vel` is remapped to `/{ns}/cmd_vel_stamped_raw`
     instead of `/{ns}/cmd_vel_stamped`.
  2. This node subscribes to `_raw`, publishes to `/{ns}/cmd_vel_stamped`.
  3. `twist_bridge` and downstream consumers are unchanged — they still
     read `/{ns}/cmd_vel_stamped` which is now the supervised output.

Real-robot safety
-----------------
All inputs to the clamp decision (`LaserScan`, `TwistStamped`) come from
sensors/topics that exist identically on a real Go2W. No sim-specific
dependencies. `max_decel` matches the CMU stack's `maxAccel` so the clamp
is conservative vs the locomotion controller's braking capacity.
"""
from __future__ import annotations

import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class VelocitySafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("velocity_safety_supervisor")

        self.declare_parameter("max_linear_speed_m_s", 0.4)
        self.declare_parameter("max_decel_m_s2", 2.0)
        self.declare_parameter("safety_margin_m", 0.10)
        self.declare_parameter("forward_arc_half_rad", 1.047)
        self.declare_parameter("min_valid_range_m", 0.05)
        self.declare_parameter("stale_cmd_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("scan_topic", "/robot/scan_3d")
        self.declare_parameter("cmd_in_topic", "/robot/cmd_vel_stamped_raw")
        self.declare_parameter("cmd_out_topic", "/robot/cmd_vel_stamped")
        self.declare_parameter("debug", False)

        p = self.get_parameter
        self._v_max = float(p("max_linear_speed_m_s").value)
        self._a_max = float(p("max_decel_m_s2").value)
        self._d_safe = float(p("safety_margin_m").value)
        self._arc = float(p("forward_arc_half_rad").value)
        self._min_range = float(p("min_valid_range_m").value)
        self._stale_timeout = float(p("stale_cmd_timeout_sec").value)
        self._rate_hz = float(p("publish_rate_hz").value)
        scan_topic = str(p("scan_topic").value)
        cmd_in_topic = str(p("cmd_in_topic").value)
        cmd_out_topic = str(p("cmd_out_topic").value)
        self._debug = bool(p("debug").value)

        self._latest_cmd: Optional[TwistStamped] = None
        self._latest_cmd_t: float = 0.0
        self._nearest_forward_m: float = float("inf")
        self._nearest_overall_m: float = float("inf")
        self._last_scan_t: float = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(TwistStamped, cmd_in_topic, self._on_cmd, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_out_topic, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f"velocity_safety_supervisor up: v_max={self._v_max:.2f} m/s, "
            f"a_max={self._a_max:.2f} m/s², d_safe={self._d_safe:.2f} m, "
            f"arc=±{math.degrees(self._arc):.0f}°, "
            f"in={cmd_in_topic}, out={cmd_out_topic}, scan={scan_topic}"
        )

    # ── Subscriptions ────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan) -> None:
        """Compute nearest forward-cone range from the 2D scan."""
        n = len(msg.ranges)
        if n == 0:
            return

        max_valid = msg.range_max if msg.range_max > 0 else float("inf")
        nearest_fwd = float("inf")
        nearest_any = float("inf")

        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r):
                continue
            if r < self._min_range or r > max_valid:
                continue
            if r < nearest_any:
                nearest_any = r
            angle = msg.angle_min + i * msg.angle_increment
            # Wrap angle to [-π, π] so we can compare against forward arc
            # regardless of whether the scan is 0..2π or −π..π.
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            if abs(angle) <= self._arc and r < nearest_fwd:
                nearest_fwd = r

        self._nearest_forward_m = nearest_fwd
        self._nearest_overall_m = nearest_any
        self._last_scan_t = time.monotonic()

    def _on_cmd(self, msg: TwistStamped) -> None:
        self._latest_cmd = msg
        self._latest_cmd_t = time.monotonic()

    # ── Control tick ──────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()

        # No recent command → hold zero (hard stop).
        if self._latest_cmd is None or (now - self._latest_cmd_t) > self._stale_timeout:
            self._publish_zero()
            return

        cmd = self._latest_cmd
        vx_raw = float(cmd.twist.linear.x)
        vy_raw = float(cmd.twist.linear.y)
        wz_raw = float(cmd.twist.angular.z)

        cmd_speed = math.hypot(vx_raw, vy_raw)
        if cmd_speed < 1e-6:
            # Already zero — pass through angular only (in-place rotation).
            self._publish(0.0, 0.0, wz_raw, cmd.header.stamp, cmd.header.frame_id)
            return

        # Choose which nearest-range to use based on sign of forward cmd:
        #  - forward (vx > 0): use forward-cone nearest
        #  - reverse (vx < 0): no rear-cone available in a 2D scan slice, so
        #    we fall back to the overall nearest — reverse is conservative
        #    because stuck-recovery maneuvers are short.
        if vx_raw >= 0.0:
            d = self._nearest_forward_m
        else:
            d = self._nearest_overall_m

        # Scan staleness guard — if the scan hasn't updated for > 0.5 s,
        # treat it as "no obstacle info" and hard-stop (safer than passing
        # through the cmd_vel and hoping).
        if (now - self._last_scan_t) > 0.5:
            self._publish_zero()
            if self._debug:
                self.get_logger().warning("scan stale — emitting zero")
            return

        d_reserve = max(0.0, d - self._d_safe)
        v_cap = math.sqrt(2.0 * self._a_max * d_reserve)
        v_limit = min(v_cap, self._v_max)

        # Scale vx, vy by the ratio needed to bring speed under v_limit,
        # preserving direction. Angular velocity scales with the same
        # ratio so instantaneous curvature is preserved when clamping.
        if cmd_speed > v_limit:
            scale = v_limit / cmd_speed if cmd_speed > 1e-6 else 0.0
        else:
            scale = 1.0

        vx_out = vx_raw * scale
        vy_out = vy_raw * scale
        wz_out = wz_raw * scale if scale < 1.0 else wz_raw

        self._publish(vx_out, vy_out, wz_out, cmd.header.stamp, cmd.header.frame_id)

        if self._debug and scale < 1.0:
            self.get_logger().info(
                f"clamped: v_cmd={cmd_speed:.2f} → {math.hypot(vx_out, vy_out):.2f} m/s "
                f"(d_nearest={d:.2f} m, v_cap={v_cap:.2f} m/s, scale={scale:.2f})"
            )

    # ── Publishing helpers ───────────────────────────────────────────
    def _publish(
        self,
        vx: float,
        vy: float,
        wz: float,
        stamp,
        frame_id: str,
    ) -> None:
        out = TwistStamped()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.angular.z = wz
        self._cmd_pub.publish(out)

    def _publish_zero(self) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        self._cmd_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = VelocitySafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
