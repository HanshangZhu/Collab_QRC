#!/usr/bin/env python3
"""Route Go2W Gazebo velocity commands to legged or wheel motion."""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


DEFAULT_RAMP_FORCE_WHEEL_MODES = frozenset({"ramp", "platform"})


@dataclass
class MotionThresholds:
    idle_linear: float
    idle_lateral: float
    idle_angular: float
    wheel_linear: float
    wheel_lateral: float
    wheel_angular: float
    wheel_curvature: float


def ramp_force_active(
    *,
    goal_xy: tuple[float, float] | None,
    goal_rx_sec: float | None,
    now_sec: float,
    stale_sec: float,
    min_x: float,
    max_x: float,
    max_abs_y: float,
) -> bool:
    if goal_xy is None or goal_rx_sec is None:
        return False
    if now_sec - goal_rx_sec > stale_sec:
        return False
    goal_x, goal_y = goal_xy
    return min_x <= goal_x <= max_x and abs(goal_y) <= max_abs_y


def normalized_mode_set(values) -> set[str]:
    if isinstance(values, str):
        raw_values = values.split(",")
    else:
        raw_values = values
    modes: set[str] = set()
    for value in raw_values:
        mode = str(value).strip().lower()
        if mode:
            modes.add(mode)
    return modes


def limit_ramp_legged_values(
    *,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    max_vx: float,
    max_abs_wz: float,
) -> tuple[float, float, float]:
    vx = min(max(0.0, float(linear_x)), max(0.0, float(max_vx)))
    wz_limit = max(0.0, float(max_abs_wz))
    wz = max(-wz_limit, min(wz_limit, float(angular_z)))
    return vx, 0.0, wz


def limit_ramp_wheel_values(
    *,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    max_vx: float,
    max_abs_wz: float,
) -> tuple[float, float, float]:
    vx = min(max(0.0, float(linear_x)), max(0.0, float(max_vx)))
    wz_limit = max(0.0, float(max_abs_wz))
    wz = max(-wz_limit, min(wz_limit, float(angular_z)))
    return vx, 0.0, wz


def limit_ramp_wheel_values_for_recency(
    *,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    cmd_recent: bool,
    cmd_idle: bool,
    max_vx: float,
    max_abs_wz: float,
) -> tuple[float, float, float]:
    if not cmd_recent or cmd_idle:
        return 0.0, 0.0, 0.0
    return limit_ramp_wheel_values(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        max_vx=max_vx,
        max_abs_wz=max_abs_wz,
    )


class Go2WHybridCmdRouter(Node):
    def __init__(self) -> None:
        super().__init__("go2w_hybrid_cmd_router")

        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("legged_topic", "cmd_vel_legged")
        self.declare_parameter("wheel_command_topic", "wheel_velocity_controller/commands")
        self.declare_parameter("status_topic", "mobility_mode")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.50)
        self.declare_parameter("idle_linear_threshold", 0.02)
        self.declare_parameter("idle_lateral_threshold", 0.02)
        self.declare_parameter("idle_angular_threshold", 0.05)
        self.declare_parameter("wheel_linear_threshold", 0.18)
        self.declare_parameter("wheel_lateral_threshold", 0.05)
        self.declare_parameter("wheel_angular_threshold", 0.20)
        self.declare_parameter("wheel_curvature_threshold", 0.45)
        self.declare_parameter("wheel_mode_hold_sec", 0.6)
        self.declare_parameter("legged_mode_hold_sec", 0.6)
        # Curvature override: when requested mode is legged AND the commanded
        # κ=|ω|/|v| exceeds this, bypass mode hold and switch immediately.
        # Fixes the "wheel mode U-turn" failure: robot was cruising east in
        # wheel mode; astar replanned to a goal ~180° behind; it commanded
        # (v≈0.05, ω≈0.35) → κ=7/m, clearly legged; but wheel_mode_hold_sec
        # (0.6–1.2 s) held wheel mode → wheels drew a wide skid-steer U-turn
        # and hit the wall. With this override, any κ above `wheel_curvature_threshold × 2`
        # forces immediate legged mode so CHAMP pivots in place.
        self.declare_parameter("legged_override_curvature", 1.0)
        # NOTE (iter 7): wheel_pivot / wheel_curve modes were removed.
        # Rationale: CHAMP's leg joints remain compliant under the impedance
        # controller even when cmd_vel_legged=0, so any lateral scrub force
        # from a wheel skid-steer pivot propagates up the calf/thigh/hip
        # chain and causes body oscillation → the "spinning circles"
        # failure mode we observed. CHAMP already supports an in-place
        # rotation gait (triggered by cmd_vel = (0,0,ω)) that does this
        # deterministically with zero ground slip. Legs for turns, wheels
        # for forward straight-line highways only.
        self.declare_parameter("wheel_radius_m", 0.09)
        self.declare_parameter("wheel_track_m", 0.40)
        self.declare_parameter("wheel_max_angular_speed", 8.5)
        self.declare_parameter("wheel_joint_signs", [1.0, 1.0, 1.0, 1.0])
        # Sustain hysteresis: cmd_vel must satisfy wheel-eligible criteria
        # CONTINUOUSLY for this many seconds before switching to wheel
        # mode. Disengagement is immediate. Filters out brief cmd peaks
        # at goal-switch moments that would power the wheels and then
        # leave them coasting when cmd drops to 0.
        self.declare_parameter("wheel_engage_sustain_sec", 0.5)
        # In legged/idle mode, mirror actual wheel ω back as the velocity
        # setpoint so the velocity actuator's brake torque is zero (error=0)
        # and the wheels can freewheel under the body's leg-gait motion. With
        # the previous default (cmd=0 in legged mode), kv=5 generates up to
        # ~10 N·m brake force opposing residual body motion → wheel skids
        # against the ground. Subscribe to joint_states to read actual ω.
        # Names match the MJCF foot_joint convention ([FL,FR,RL,RR]_foot_joint
        # — Robot A is unprefixed; Robot B is b_*-prefixed and skipped).
        # 2026-05-10: was "/mujoco_sim/joint_states" but the actual publisher
        # is `robot_joint_states_controller` (JointStateBroadcaster) which
        # publishes to its own controller_manager namespace = `/<ns>/joint_states`.
        # The old absolute default had publisher_count=0 → router never read
        # actual ω → published [0,0,0,0] → kv=5 actuator brake-locked the
        # wheels in legged mode (THE wheel-skid bug). Default is now relative
        # so it picks up `/<ns>/joint_states` automatically.
        self.declare_parameter("wheel_state_topic", "joint_states")
        self.declare_parameter("wheel_joint_names", [
            "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"
        ])
        self.declare_parameter("freewheel_in_legged", True)
        self.declare_parameter("ramp_force_legged_enabled", False)
        self.declare_parameter("ramp_force_wheel_enabled", False)
        self.declare_parameter("ramp_goal_topic", "ramp_ascent_goal")
        self.declare_parameter("ramp_goal_mode_topic", "")
        self.declare_parameter("ramp_goal_stale_sec", 1.5)
        self.declare_parameter("ramp_goal_mode_sync_tolerance_sec", 0.25)
        self.declare_parameter("ramp_force_wheel_modes", ["ramp", "platform"])
        self.declare_parameter("ramp_force_min_goal_x", -1.0e9)
        self.declare_parameter("ramp_force_max_goal_x", 1.0e9)
        self.declare_parameter("ramp_force_max_abs_goal_y", 1.0e9)
        self.declare_parameter("ramp_force_max_vx_mps", 0.17)
        self.declare_parameter("ramp_force_max_yaw_rate_rps", 0.35)
        # Pitch-triggered force-wheel: when the body pitches past a threshold
        # (robot physically on a slope), hold sustained wheel mode and floor
        # the forward speed so the wheels drive briskly UP the incline instead
        # of stalling. Goal-independent (unlike ramp_goal-based force-wheel),
        # so it works regardless of where CFPA2's goal is. Reads pitch from an
        # odometry source.
        self.declare_parameter("pitch_force_wheel_enabled", False)
        self.declare_parameter("pitch_force_wheel_threshold_rad", 0.070)  # ~4 deg (filtered)
        self.declare_parameter("pitch_ema_alpha", 0.15)  # EMA on pitch: rejects gait bob
        self.declare_parameter("pitch_climb_vx_mps", 0.45)
        self.declare_parameter("pitch_climb_max_yaw_rate_rps", 0.40)
        self.declare_parameter("pitch_odom_topic", "odom/nav")
        self.declare_parameter("pitch_stale_sec", 0.5)

        input_topic = str(self.get_parameter("input_topic").value)
        legged_topic = str(self.get_parameter("legged_topic").value)
        wheel_command_topic = str(self.get_parameter("wheel_command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.cmd_timeout_sec = max(0.0, float(self.get_parameter("cmd_timeout_sec").value))
        self.thresholds = MotionThresholds(
            idle_linear=max(0.0, float(self.get_parameter("idle_linear_threshold").value)),
            idle_lateral=max(0.0, float(self.get_parameter("idle_lateral_threshold").value)),
            idle_angular=max(0.0, float(self.get_parameter("idle_angular_threshold").value)),
            wheel_linear=max(0.0, float(self.get_parameter("wheel_linear_threshold").value)),
            wheel_lateral=max(0.0, float(self.get_parameter("wheel_lateral_threshold").value)),
            wheel_angular=max(0.0, float(self.get_parameter("wheel_angular_threshold").value)),
            wheel_curvature=max(0.0, float(self.get_parameter("wheel_curvature_threshold").value)),
        )
        self.mode_hold_sec = {
            "wheel": max(0.0, float(self.get_parameter("wheel_mode_hold_sec").value)),
            "legged": max(0.0, float(self.get_parameter("legged_mode_hold_sec").value)),
        }
        self.legged_override_curvature = max(
            0.0, float(self.get_parameter("legged_override_curvature").value)
        )
        self.wheel_radius_m = max(1e-4, float(self.get_parameter("wheel_radius_m").value))
        self.wheel_track_m = max(1e-4, float(self.get_parameter("wheel_track_m").value))
        self.wheel_max_angular_speed = max(
            0.0, float(self.get_parameter("wheel_max_angular_speed").value)
        )

        raw_signs = list(self.get_parameter("wheel_joint_signs").value)
        self.wheel_joint_signs = [float(value) for value in raw_signs[:4]]
        if len(self.wheel_joint_signs) != 4:
            raise ValueError("wheel_joint_signs must contain exactly four entries")

        self.wheel_engage_sustain_sec = max(
            0.0, float(self.get_parameter("wheel_engage_sustain_sec").value)
        )

        self._last_cmd = Twist()
        self._last_cmd_time_sec: float | None = None
        self._active_mode = "idle"
        self._last_mode_change_sec: float | None = None
        # Tracks when cmd_vel first became wheel-eligible (None if not).
        # Reset on any below-threshold tick. Sustain hysteresis condition.
        self._wheel_eligible_since_sec: float | None = None

        self.wheel_joint_names = [str(v) for v in self.get_parameter("wheel_joint_names").value][:4]
        self.freewheel_in_legged = bool(self.get_parameter("freewheel_in_legged").value)
        self.ramp_force_legged_enabled = bool(
            self.get_parameter("ramp_force_legged_enabled").value
        )
        self.ramp_force_wheel_enabled = bool(
            self.get_parameter("ramp_force_wheel_enabled").value
        )
        self.ramp_goal_stale_sec = max(
            0.1, float(self.get_parameter("ramp_goal_stale_sec").value)
        )
        self.ramp_goal_mode_topic = str(
            self.get_parameter("ramp_goal_mode_topic").value
        ).strip()
        self.ramp_goal_mode_sync_tolerance_sec = max(
            0.0, float(self.get_parameter("ramp_goal_mode_sync_tolerance_sec").value)
        )
        self.ramp_force_wheel_modes = normalized_mode_set(
            self.get_parameter("ramp_force_wheel_modes").value
        )
        if not self.ramp_force_wheel_modes:
            self.ramp_force_wheel_modes = set(DEFAULT_RAMP_FORCE_WHEEL_MODES)
        self.ramp_force_min_goal_x = float(
            self.get_parameter("ramp_force_min_goal_x").value
        )
        self.ramp_force_max_goal_x = float(
            self.get_parameter("ramp_force_max_goal_x").value
        )
        self.ramp_force_max_abs_goal_y = max(
            0.0, float(self.get_parameter("ramp_force_max_abs_goal_y").value)
        )
        self.ramp_force_max_vx_mps = max(
            0.0, float(self.get_parameter("ramp_force_max_vx_mps").value)
        )
        self.ramp_force_max_yaw_rate_rps = max(
            0.0, float(self.get_parameter("ramp_force_max_yaw_rate_rps").value)
        )
        self._ramp_goal_xy: tuple[float, float] | None = None
        self._ramp_goal_rx_sec: float | None = None
        self._ramp_goal_mode: str | None = None
        self._ramp_goal_mode_rx_sec: float | None = None

        self.pitch_force_wheel_enabled = bool(
            self.get_parameter("pitch_force_wheel_enabled").value
        )
        self.pitch_force_wheel_threshold_rad = abs(
            float(self.get_parameter("pitch_force_wheel_threshold_rad").value)
        )
        self.pitch_ema_alpha = min(1.0, max(0.01,
            float(self.get_parameter("pitch_ema_alpha").value)))
        self.pitch_climb_vx_mps = max(
            0.0, float(self.get_parameter("pitch_climb_vx_mps").value)
        )
        self.pitch_climb_max_yaw_rate_rps = max(
            0.0, float(self.get_parameter("pitch_climb_max_yaw_rate_rps").value)
        )
        self.pitch_stale_sec = max(
            0.05, float(self.get_parameter("pitch_stale_sec").value)
        )
        self._pitch_rad: float = 0.0
        self._pitch_rx_sec: float | None = None
        # Latest measured ω for [FL,FR,RL,RR]_foot_joint (rad/s). Defaults
        # to zeros so the first few ticks (before joint_states arrives)
        # behave like the legacy "cmd=0 in legged" path.
        self._latest_wheel_vels: list[float] = [0.0, 0.0, 0.0, 0.0]

        self.create_subscription(Twist, input_topic, self._cmd_cb, 10)
        wheel_state_topic = str(self.get_parameter("wheel_state_topic").value)
        self.create_subscription(JointState, wheel_state_topic, self._joint_state_cb, 10)
        if self.ramp_force_legged_enabled or self.ramp_force_wheel_enabled:
            ramp_goal_topic = str(self.get_parameter("ramp_goal_topic").value)
            self.create_subscription(PointStamped, ramp_goal_topic, self._ramp_goal_cb, 10)
            if self.ramp_goal_mode_topic:
                self.create_subscription(
                    String, self.ramp_goal_mode_topic, self._ramp_goal_mode_cb, 10
                )
        if self.pitch_force_wheel_enabled:
            pitch_odom_topic = str(self.get_parameter("pitch_odom_topic").value)
            self.create_subscription(Odometry, pitch_odom_topic, self._odom_cb, 10)
        self._legged_pub = self.create_publisher(Twist, legged_topic, 10)
        self._wheel_pub = self.create_publisher(Float64MultiArray, wheel_command_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            "Go2W hybrid cmd router started: "
            f"{input_topic} -> {legged_topic} | {wheel_command_topic}"
        )
        if self.pitch_force_wheel_enabled:
            self.get_logger().info(
                "Pitch force-wheel enabled: "
                f"odom={self.get_parameter('pitch_odom_topic').value} "
                f"|pitch|>{math.degrees(self.pitch_force_wheel_threshold_rad):.1f}deg "
                f"-> sustained wheel, climb_vx={self.pitch_climb_vx_mps:.2f} "
                f"|wz|<={self.pitch_climb_max_yaw_rate_rps:.2f}"
            )
        if self.ramp_force_legged_enabled:
            self.get_logger().info(
                "Ramp force-legged enabled: "
                f"goal={self.get_parameter('ramp_goal_topic').value} "
                f"x=[{self.ramp_force_min_goal_x:.1f},{self.ramp_force_max_goal_x:.1f}] "
                f"|y|<={self.ramp_force_max_abs_goal_y:.1f} "
                f"vx<={self.ramp_force_max_vx_mps:.2f}"
            )
        if self.ramp_force_wheel_enabled:
            self.get_logger().info(
                "Ramp force-wheel enabled: "
                f"goal={self.get_parameter('ramp_goal_topic').value} "
                f"mode_topic={self.ramp_goal_mode_topic or 'disabled'} "
                f"wheel_modes={sorted(self.ramp_force_wheel_modes)} "
                f"x=[{self.ramp_force_min_goal_x:.1f},{self.ramp_force_max_goal_x:.1f}] "
                f"|y|<={self.ramp_force_max_abs_goal_y:.1f} "
                f"vx<={self.ramp_force_max_vx_mps:.2f} "
                f"|wz|<={self.ramp_force_max_yaw_rate_rps:.2f}"
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cmd_cb(self, msg: Twist) -> None:
        # Event-driven publish: as soon as MPPI gives us a fresh cmd_vel,
        # forward it immediately. The 50ms timer-wait that used to sit
        # between the callback and the publish was the dominant single
        # source of brake-latency in the chain (MPPI publishes at 20 Hz,
        # router timer was 20 Hz too → up to 50ms additional delay).
        # CHAMP caches cmd_vel anyway in its own 200 Hz loop, so a faster
        # publish rate from us is purely upside.
        self._last_cmd = msg
        self._last_cmd_time_sec = self._now_sec()
        self._tick()

    def _ramp_goal_cb(self, msg: PointStamped) -> None:
        goal = (float(msg.point.x), float(msg.point.y))
        if not all(math.isfinite(v) for v in goal):
            return
        self._ramp_goal_xy = goal
        self._ramp_goal_rx_sec = self._now_sec()

    def _ramp_goal_mode_cb(self, msg: String) -> None:
        mode = str(msg.data).strip().lower()
        if not mode:
            return
        self._ramp_goal_mode = mode
        self._ramp_goal_mode_rx_sec = self._now_sec()

    def _joint_state_cb(self, msg: JointState) -> None:
        # Cache latest ω for the four wheel joints. Skip Robot B (b_-prefix).
        for nm, vel in zip(msg.name, msg.velocity):
            if nm in self.wheel_joint_names:
                self._latest_wheel_vels[self.wheel_joint_names.index(nm)] = float(vel)

    def _is_recent(self, now_sec: float) -> bool:
        return self._last_cmd_time_sec is not None and (now_sec - self._last_cmd_time_sec) <= self.cmd_timeout_sec

    def _is_idle(self, cmd: Twist) -> bool:
        return (
            abs(float(cmd.linear.x)) < self.thresholds.idle_linear
            and abs(float(cmd.linear.y)) < self.thresholds.idle_lateral
            and abs(float(cmd.angular.z)) < self.thresholds.idle_angular
        )

    def _requested_mode(self, cmd: Twist, now_sec: float) -> tuple[str, float]:
        """Return (mode, curvature) — curvature surfaced so the selector can
        bypass mode hold for emergency U-turns (high κ legged demands)."""
        if self._ramp_force_wheel_active(now_sec) or self._pitch_force_wheel_active(now_sec):
            self._wheel_eligible_since_sec = now_sec - self.wheel_engage_sustain_sec
            linear_x = abs(float(cmd.linear.x))
            angular_z = abs(float(cmd.angular.z))
            curvature = angular_z / max(linear_x, 0.05)
            return ("wheel", curvature)

        if not self._is_recent(now_sec) or self._is_idle(cmd):
            return ("idle", 0.0)

        raw_linear_x = float(cmd.linear.x)
        linear_x = abs(raw_linear_x)
        linear_y = abs(float(cmd.linear.y))
        angular_z = abs(float(cmd.angular.z))
        curvature = angular_z / max(linear_x, 0.05)
        if self._ramp_force_active(now_sec):
            self._wheel_eligible_since_sec = None
            return ("legged", curvature)

        # Sustain hysteresis: cmd_vel must satisfy wheel-eligible criteria
        # CONTINUOUSLY for `wheel_engage_sustain_sec` before we engage
        # wheel mode. Disengagement is immediate (one tick of below-
        # threshold cmd → back to legged). Filters out brief cmd peaks at
        # goal-switch moments that would otherwise power the wheels and
        # then leave them coasting when cmd drops to 0 (wheel velocity
        # controller has soft kv → wheels coast 5+ s before stopping).
        wheel_eligible = (
            raw_linear_x > 0
            and linear_x >= self.thresholds.wheel_linear
            and linear_y <= self.thresholds.wheel_lateral
            and angular_z <= self.thresholds.wheel_angular
            and curvature <= self.thresholds.wheel_curvature
        )
        if wheel_eligible:
            if self._wheel_eligible_since_sec is None:
                self._wheel_eligible_since_sec = now_sec
            sustained = now_sec - self._wheel_eligible_since_sec
            if sustained >= self.wheel_engage_sustain_sec:
                return ("wheel", curvature)
        else:
            self._wheel_eligible_since_sec = None

        return ("legged", curvature)

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        # body pitch from the orientation quaternion (rotation about y).
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        # EMA low-pass: the legged gait bobs the body ±3-5° around 0, which
        # would false-trigger force-wheel on flat ground. Filtering averages
        # the oscillation toward 0 while preserving a ramp's SUSTAINED tilt, so
        # a modest threshold cleanly separates "on a slope" from "gait bob".
        self._pitch_rad = (
            self.pitch_ema_alpha * pitch
            + (1.0 - self.pitch_ema_alpha) * self._pitch_rad
        )
        self._pitch_rx_sec = self._now_sec()

    def _pitch_force_wheel_active(self, now_sec: float) -> bool:
        if not self.pitch_force_wheel_enabled or self._pitch_rx_sec is None:
            return False
        if now_sec - self._pitch_rx_sec > self.pitch_stale_sec:
            return False
        return abs(self._pitch_rad) > self.pitch_force_wheel_threshold_rad

    def _ramp_force_active(self, now_sec: float) -> bool:
        if not self.ramp_force_legged_enabled:
            return False
        return ramp_force_active(
            goal_xy=self._ramp_goal_xy,
            goal_rx_sec=self._ramp_goal_rx_sec,
            now_sec=now_sec,
            stale_sec=self.ramp_goal_stale_sec,
            min_x=self.ramp_force_min_goal_x,
            max_x=self.ramp_force_max_goal_x,
            max_abs_y=self.ramp_force_max_abs_goal_y,
        )

    def _ramp_force_wheel_active(self, now_sec: float) -> bool:
        if not self.ramp_force_wheel_enabled:
            return False
        if not self._ramp_force_wheel_mode_allows(now_sec):
            return False
        return ramp_force_active(
            goal_xy=self._ramp_goal_xy,
            goal_rx_sec=self._ramp_goal_rx_sec,
            now_sec=now_sec,
            stale_sec=self.ramp_goal_stale_sec,
            min_x=self.ramp_force_min_goal_x,
            max_x=self.ramp_force_max_goal_x,
            max_abs_y=self.ramp_force_max_abs_goal_y,
        )

    def _ramp_force_wheel_mode_allows(self, now_sec: float) -> bool:
        mode_topic = getattr(self, "ramp_goal_mode_topic", "")
        mode = getattr(self, "_ramp_goal_mode", None)
        mode_rx_sec = getattr(self, "_ramp_goal_mode_rx_sec", None)
        if mode is None or mode_rx_sec is None:
            return not bool(mode_topic)
        if now_sec - mode_rx_sec > self.ramp_goal_stale_sec:
            return not bool(mode_topic)

        goal_rx_sec = getattr(self, "_ramp_goal_rx_sec", None)
        if mode_topic and goal_rx_sec is not None:
            if mode_rx_sec + 1.0e-6 < goal_rx_sec:
                return False
            sync_tolerance = getattr(
                self, "ramp_goal_mode_sync_tolerance_sec", 0.25
            )
            if mode_rx_sec - goal_rx_sec > sync_tolerance:
                return False

        allowed_modes = getattr(
            self, "ramp_force_wheel_modes", DEFAULT_RAMP_FORCE_WHEEL_MODES
        )
        return str(mode).strip().lower() in allowed_modes

    def _select_mode(self, requested_mode: str, curvature: float,
                     now_sec: float) -> str:
        if requested_mode == self._active_mode:
            return requested_mode

        if requested_mode == "wheel" and (
            self._ramp_force_wheel_active(now_sec)
            or self._pitch_force_wheel_active(now_sec)
        ):
            return requested_mode

        if requested_mode == "legged" and self._ramp_force_active(now_sec):
            return requested_mode

        # Emergency bypass: if astar is demanding legged with high curvature
        # (e.g. U-turn, pivot-in-place for 180° heading flip), skip mode hold.
        # Without this, a robot cruising in wheel mode east that gets a new
        # goal behind it would execute a wide wheel skid-steer U-turn for
        # the full hold window and crash into walls before switching to the
        # pivot-in-place CHAMP gait that astar actually wants.
        if (
            requested_mode == "legged"
            and self._active_mode == "wheel"
            and curvature >= self.legged_override_curvature
        ):
            return requested_mode

        if self._active_mode in self.mode_hold_sec and self._last_mode_change_sec is not None:
            held_for = now_sec - self._last_mode_change_sec
            if held_for < self.mode_hold_sec[self._active_mode]:
                return self._active_mode
        return requested_mode

    def _wheel_command(self, cmd: Twist) -> Float64MultiArray:
        half_track = 0.5 * self.wheel_track_m
        angular = float(cmd.angular.z)
        left_linear = float(cmd.linear.x) - (angular * half_track)
        right_linear = float(cmd.linear.x) + (angular * half_track)
        left_omega = left_linear / self.wheel_radius_m
        right_omega = right_linear / self.wheel_radius_m
        left_omega = max(-self.wheel_max_angular_speed, min(self.wheel_max_angular_speed, left_omega))
        right_omega = max(-self.wheel_max_angular_speed, min(self.wheel_max_angular_speed, right_omega))
        msg = Float64MultiArray()
        # Wheel order is [front_left, front_right, rear_left, rear_right].
        msg.data = [
            self.wheel_joint_signs[0] * left_omega,
            self.wheel_joint_signs[1] * right_omega,
            self.wheel_joint_signs[2] * left_omega,
            self.wheel_joint_signs[3] * right_omega,
        ]
        return msg

    def _tick(self) -> None:
        now_sec = self._now_sec()
        requested_mode, requested_curv = self._requested_mode(self._last_cmd, now_sec)
        selected_mode = self._select_mode(requested_mode, requested_curv, now_sec)

        if selected_mode != self._active_mode:
            self._active_mode = selected_mode
            self._last_mode_change_sec = now_sec
            self.get_logger().info(f"Mobility mode switched to {selected_mode}")

        legged_cmd = Twist()
        wheel_cmd = Float64MultiArray()
        # Default freewheel: mirror current actual ω back as setpoint so the
        # velocity actuator's brake torque is zero. Without this, kv≈50 with
        # cmd=0 generates ±15 N·m brake force opposing residual body motion
        # under leg gait → wheel skids on the ground in legged/idle mode.
        # First few ticks before joint_states arrives use the legacy [0,0,0,0]
        # path (latest_wheel_vels is initialised to zeros).
        if self.freewheel_in_legged:
            wheel_cmd.data = list(self._latest_wheel_vels)
        else:
            wheel_cmd.data = [0.0, 0.0, 0.0, 0.0]

        if self._active_mode == "wheel":
            if self._ramp_force_wheel_active(now_sec):
                vx, vy, wz = limit_ramp_wheel_values_for_recency(
                    linear_x=float(self._last_cmd.linear.x),
                    linear_y=float(self._last_cmd.linear.y),
                    angular_z=float(self._last_cmd.angular.z),
                    cmd_recent=self._is_recent(now_sec),
                    cmd_idle=self._is_idle(self._last_cmd),
                    max_vx=self.ramp_force_max_vx_mps,
                    max_abs_wz=self.ramp_force_max_yaw_rate_rps,
                )
                ramp_cmd = Twist()
                ramp_cmd.linear.x = vx
                ramp_cmd.linear.y = vy
                ramp_cmd.angular.z = wz
                wheel_cmd = self._wheel_command(ramp_cmd)
            elif self._pitch_force_wheel_active(now_sec):
                # On a slope: floor the forward speed so the wheels drive
                # briskly up the incline instead of stalling at MPPI's cautious
                # creep. Only when MPPI still intends forward + a fresh cmd —
                # never force forward when it wants to stop/reverse. Cap yaw so
                # the climb stays roughly straight (oblique climbs risk tipping).
                raw_vx = float(self._last_cmd.linear.x)
                if self._is_recent(now_sec) and raw_vx > self.thresholds.idle_linear:
                    vx = max(raw_vx, self.pitch_climb_vx_mps)
                else:
                    vx = max(0.0, raw_vx)
                wz = max(-self.pitch_climb_max_yaw_rate_rps,
                         min(self.pitch_climb_max_yaw_rate_rps,
                             float(self._last_cmd.angular.z)))
                climb_cmd = Twist()
                climb_cmd.linear.x = vx
                climb_cmd.angular.z = wz
                wheel_cmd = self._wheel_command(climb_cmd)
            else:
                wheel_cmd = self._wheel_command(self._last_cmd)
        elif self._active_mode == "legged":
            if self._ramp_force_active(now_sec):
                vx, vy, wz = limit_ramp_legged_values(
                    linear_x=float(self._last_cmd.linear.x),
                    linear_y=float(self._last_cmd.linear.y),
                    angular_z=float(self._last_cmd.angular.z),
                    max_vx=self.ramp_force_max_vx_mps,
                    max_abs_wz=self.ramp_force_max_yaw_rate_rps,
                )
                legged_cmd.linear.x = vx
                legged_cmd.linear.y = vy
                legged_cmd.angular.z = wz
            else:
                legged_cmd = self._last_cmd

        self._legged_pub.publish(legged_cmd)
        self._wheel_pub.publish(wheel_cmd)

        status = String()
        status.data = self._active_mode
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2WHybridCmdRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
