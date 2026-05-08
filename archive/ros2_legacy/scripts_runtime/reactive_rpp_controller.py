#!/usr/bin/env python3
"""Reactive Regulated Pure Pursuit controller — replaces CMU localPlanner + pathFollower.

Reads FAR's intermediate waypoints on /{ns}/way_point, pursues them with
pure-pursuit, and decelerates near obstacles using the raw LaserScan. No
costmap, no voxelization, no path library, no forward/reverse mode switching.

Architecture:
  FAR (global V-graph) → /robot/way_point (PointStamped)
       ↓
  THIS NODE ← /robot/scan_3d (LaserScan) + /robot/odom/nav (Odometry)
       ↓
  /robot/cmd_vel (Twist) → go2w_hybrid_cmd_router → CHAMP

Key properties:
  - ALWAYS drives forward toward the waypoint. No reverse mode.
  - Decelerates proportionally to nearest-obstacle clearance in a body-
    width corridor ahead (same physics model as velocity_safety_supervisor).
  - If stuck (commanding forward but actual velocity near zero for > 2 s),
    executes a short backup + rotate recovery, then resumes.
  - 20 Hz control loop, ~150 lines of logic. Real-robot safe — only reads
    LaserScan + Odometry + PointStamped.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ReactiveRPPController(Node):
    def __init__(self) -> None:
        super().__init__("reactive_rpp_controller")

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("lookahead_dist", 0.6)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_decel", 2.0)
        self.declare_parameter("safety_margin_m", 0.10)
        self.declare_parameter("body_half_width", 0.25)
        # With the L1 at the chin (x=0.29, z=-0.04), the forward sector
        # is unobstructed — no body parts ahead. Safe to sense close.
        self.declare_parameter("min_valid_range", 0.05)
        self.declare_parameter("control_rate", 20.0)
        # Stuck detection
        self.declare_parameter("stuck_cmd_thresh", 0.08)
        self.declare_parameter("stuck_vel_thresh", 0.03)
        self.declare_parameter("stuck_duration_sec", 2.0)
        self.declare_parameter("recovery_backup_m", 0.4)
        self.declare_parameter("recovery_rotate_rad", 1.0)

        p = self.get_parameter
        self._v_max = float(p("max_linear_speed").value)
        self._w_max = float(p("max_angular_speed").value)
        self._lookahead = float(p("lookahead_dist").value)
        self._goal_tol = float(p("goal_tolerance").value)
        self._a_max = float(p("max_decel").value)
        self._d_safe = float(p("safety_margin_m").value)
        self._body_hw = float(p("body_half_width").value)
        self._min_range = float(p("min_valid_range").value)
        self._rate = float(p("control_rate").value)
        self._stuck_cmd = float(p("stuck_cmd_thresh").value)
        self._stuck_vel = float(p("stuck_vel_thresh").value)
        self._stuck_dur = float(p("stuck_duration_sec").value)
        self._recovery_back = float(p("recovery_backup_m").value)
        self._recovery_rot = float(p("recovery_rotate_rad").value)

        # ── State ────────────────────────────────────────────────────
        self._goal_xy: tuple[float, float] | None = None
        self._pose_xy: tuple[float, float] = (0.0, 0.0)
        self._yaw: float = 0.0
        self._actual_vx: float = 0.0
        self._corridor_clearance: float = float("inf")
        self._last_cmd_vx: float = 0.0
        self._stuck_start: float | None = None
        self._recovering: bool = False
        self._recovery_phase: int = 0
        self._recovery_start: float = 0.0
        # Wall-stop replan: when hard-stopped by obstacle, rotate to find
        # clear corridor so FAR replans around the obstacle.
        self._wall_stop_start: float | None = None
        self._wall_rotating: bool = False

        # ── Subs / Pubs ─────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            PointStamped, "way_point", self._on_waypoint, 10
        )
        self.create_subscription(
            Odometry, "odom/nav", self._on_odom, 10
        )
        self.create_subscription(
            LaserScan, "scan_3d", self._on_scan, sensor_qos
        )
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"reactive_rpp_controller: v_max={self._v_max} w_max={self._w_max} "
            f"lookahead={self._lookahead} body_hw={self._body_hw}"
        )

    # ── Callbacks ────────────────────────────────────────────────────
    def _on_waypoint(self, msg: PointStamped) -> None:
        self._goal_xy = (float(msg.point.x), float(msg.point.y))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose_xy = (float(p.x), float(p.y))
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._actual_vx = float(msg.twist.twist.linear.x)

    def _on_scan(self, msg: LaserScan) -> None:
        """Compute nearest obstacle in the forward body-width corridor."""
        n = len(msg.ranges)
        if n == 0:
            return
        max_r = msg.range_max if msg.range_max > 0 else float("inf")
        best = float("inf")
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < self._min_range or r > max_r:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            if x > 0.0 and abs(y) < self._body_hw:
                if x < best:
                    best = x
        self._corridor_clearance = best

    # ── Control tick ─────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()
        cmd = Twist()

        # Recovery mode — execute a scripted backup + rotate.
        if self._recovering:
            elapsed = now - self._recovery_start
            if self._recovery_phase == 0:
                # Phase 0: back up
                if elapsed < self._recovery_back / 0.15:
                    cmd.linear.x = -0.15
                else:
                    self._recovery_phase = 1
                    self._recovery_start = now
            if self._recovery_phase == 1:
                # Phase 1: rotate
                if elapsed < self._recovery_rot / 0.5:
                    cmd.angular.z = 0.5
                else:
                    self._recovering = False
                    self._stuck_start = None
            self._cmd_pub.publish(cmd)
            self._last_cmd_vx = cmd.linear.x
            return

        # No goal — hold still.
        if self._goal_xy is None:
            self._cmd_pub.publish(cmd)
            self._last_cmd_vx = 0.0
            return

        # ── Pure pursuit ─────────────────────────────────────────────
        dx = self._goal_xy[0] - self._pose_xy[0]
        dy = self._goal_xy[1] - self._pose_xy[1]
        dist = math.hypot(dx, dy)

        # No hard-stop at waypoint. Pure-pursuit flows continuously;
        # approach deceleration (v_des = dist * 1.5) naturally slows near
        # the goal, and FAR updates the waypoint before the robot parks.

        # Heading error to goal.
        goal_heading = math.atan2(dy, dx)
        heading_err = _wrap_pi(goal_heading - self._yaw)

        # If goal is behind (heading error > 90°), rotate in place first.
        if abs(heading_err) > 1.3:
            cmd.angular.z = max(-self._w_max, min(self._w_max,
                                                   heading_err * 1.5))
            self._cmd_pub.publish(cmd)
            self._last_cmd_vx = 0.0
            return

        # Lookahead-based curvature.
        ld = max(self._lookahead, dist)
        curvature = 2.0 * math.sin(heading_err) / ld

        # Desired linear speed — full speed, then clamp by obstacle
        # clearance and curvature.
        v_des = self._v_max

        # Curvature regulation: slow down on tight turns.
        radius = 1.0 / max(abs(curvature), 1e-3)
        if radius < 0.9:
            v_des = min(v_des, self._v_max * (radius / 0.9))

        # Obstacle avoidance — three tiers:
        #   d < 0.25 m: hard stop + after 1 s rotate to trigger FAR replan
        #   d < 0.50 m: linear speed ramp (v_max → 0)
        #   d > 0.50 m: full speed
        stop_dist = 0.25
        slow_dist = 0.50
        d = self._corridor_clearance

        if d < stop_dist:
            now_t = time.monotonic()
            if self._wall_stop_start is None:
                self._wall_stop_start = now_t
                self._wall_rotating = False

            if not self._wall_rotating and (now_t - self._wall_stop_start) > 1.0:
                self._wall_rotating = True
                self.get_logger().info(
                    f"wall block at {d:.2f} m — rotating to trigger replan"
                )

            if self._wall_rotating:
                # Rotate toward the goal side to sweep around the obstacle.
                # FAR will see the new heading and update the waypoint.
                rot_dir = 1.0 if heading_err > 0 else -1.0
                cmd.angular.z = rot_dir * 0.6
                # If corridor clears while rotating, resume driving.
                if d > slow_dist:
                    self._wall_rotating = False
                    self._wall_stop_start = None
            else:
                pass  # v_des stays 0 via v_obs below

            v_obs = 0.0
        elif d < slow_dist:
            v_obs = self._v_max * (d - stop_dist) / (slow_dist - stop_dist)
            self._wall_stop_start = None
            self._wall_rotating = False
        else:
            v_obs = self._v_max
            self._wall_stop_start = None
            self._wall_rotating = False
        v_des = min(v_des, v_obs)

        # Approach deceleration.
        v_des = min(v_des, max(0.05, dist * 1.5))

        # Clamp.
        v_des = max(0.0, min(self._v_max, v_des))
        w_des = v_des * curvature
        w_des = max(-self._w_max, min(self._w_max, w_des))

        cmd.linear.x = v_des
        cmd.angular.z = w_des
        self._cmd_pub.publish(cmd)

        # ── Stuck detection ──────────────────────────────────────────
        self._last_cmd_vx = v_des
        if v_des > self._stuck_cmd and abs(self._actual_vx) < self._stuck_vel:
            if self._stuck_start is None:
                self._stuck_start = now
            elif (now - self._stuck_start) > self._stuck_dur:
                self.get_logger().warn(
                    f"stuck detected — recovery (v_cmd={v_des:.2f}, "
                    f"v_actual={self._actual_vx:.2f})"
                )
                self._recovering = True
                self._recovery_phase = 0
                self._recovery_start = now
        else:
            self._stuck_start = None


def main() -> None:
    rclpy.init()
    node = ReactiveRPPController()
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
