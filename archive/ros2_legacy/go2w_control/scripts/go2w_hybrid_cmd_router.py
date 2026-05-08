#!/usr/bin/env python3
"""Route Go2W Gazebo velocity commands to legged or wheel motion."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


@dataclass
class MotionThresholds:
    idle_linear: float
    idle_lateral: float
    idle_angular: float
    wheel_linear: float
    wheel_lateral: float
    wheel_angular: float
    wheel_curvature: float


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
        # the previous default (cmd=0 in legged mode), kv=50 generates up to
        # ±15 N·m brake force opposing residual body motion → wheel skids
        # against the ground. Subscribe to joint_states to read actual ω.
        # Names match the MJCF foot_joint convention ([FL,FR,RL,RR]_foot_joint
        # — Robot A is unprefixed; Robot B is b_*-prefixed and skipped).
        self.declare_parameter("wheel_state_topic", "/mujoco_sim/joint_states")
        self.declare_parameter("wheel_joint_names", [
            "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"
        ])
        self.declare_parameter("freewheel_in_legged", True)

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
        # Latest measured ω for [FL,FR,RL,RR]_foot_joint (rad/s). Defaults
        # to zeros so the first few ticks (before joint_states arrives)
        # behave like the legacy "cmd=0 in legged" path.
        self._latest_wheel_vels: list[float] = [0.0, 0.0, 0.0, 0.0]

        self.create_subscription(Twist, input_topic, self._cmd_cb, 10)
        wheel_state_topic = str(self.get_parameter("wheel_state_topic").value)
        self.create_subscription(JointState, wheel_state_topic, self._joint_state_cb, 10)
        self._legged_pub = self.create_publisher(Twist, legged_topic, 10)
        self._wheel_pub = self.create_publisher(Float64MultiArray, wheel_command_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            "Go2W hybrid cmd router started: "
            f"{input_topic} -> {legged_topic} | {wheel_command_topic}"
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
        if not self._is_recent(now_sec) or self._is_idle(cmd):
            return ("idle", 0.0)

        raw_linear_x = float(cmd.linear.x)
        linear_x = abs(raw_linear_x)
        linear_y = abs(float(cmd.linear.y))
        angular_z = abs(float(cmd.angular.z))
        curvature = angular_z / max(linear_x, 0.05)

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

    def _select_mode(self, requested_mode: str, curvature: float,
                     now_sec: float) -> str:
        if requested_mode == self._active_mode:
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
            wheel_cmd = self._wheel_command(self._last_cmd)
        elif self._active_mode == "legged":
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
