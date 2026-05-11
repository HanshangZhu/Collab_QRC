"""Leg position PD controller + wheel velocity command for Go2W.

Replaces CHAMP (C++ legged locomotion stack). For exploration we only need:
  - Legs: hold a stable standing pose via joint-space PD
  - Wheels: receive velocity commands from the nav stack

Standing pose (radians) — tuned for Go2W MJCF joint ranges:
  abduction (hip): 0.0  (range ±60°)
  thigh:           0.9  (front range -90°..+200°)
  calf:           -1.8  (range -156°..-48°)

PD gains are conservative to avoid oscillation on the 500 Hz physics step.
"""
from __future__ import annotations

import numpy as np

from .mujoco_env import MuJoCoEnv


# Standing pose repeated for [FL, FR, RL, RR] in actuator order
#   [ab, thigh, calf] × 4 legs = 12 values
_DEFAULT_STAND = np.array([
    0.0,  0.9, -1.8,   # FL
    0.0,  0.9, -1.8,   # FR
    0.0,  0.9, -1.8,   # RL
    0.0,  0.9, -1.8,   # RR
], dtype=np.float64)

# Motor torque limits from MJCF default class
_TORQUE_LIMIT_AB    = 23.7
_TORQUE_LIMIT_THIGH = 23.7
_TORQUE_LIMIT_CALF  = 45.43

_LIMITS = np.array([
    _TORQUE_LIMIT_AB, _TORQUE_LIMIT_THIGH, _TORQUE_LIMIT_CALF,
    _TORQUE_LIMIT_AB, _TORQUE_LIMIT_THIGH, _TORQUE_LIMIT_CALF,
    _TORQUE_LIMIT_AB, _TORQUE_LIMIT_THIGH, _TORQUE_LIMIT_CALF,
    _TORQUE_LIMIT_AB, _TORQUE_LIMIT_THIGH, _TORQUE_LIMIT_CALF,
], dtype=np.float64)


class LegController:
    """PD standing-pose controller for 12 leg joints.

    Call step() every physics tick to update ctrl[0:12].
    Call set_wheel_cmd() to update ctrl[12:15] from nav.
    """

    def __init__(
        self,
        env: MuJoCoEnv,
        *,
        stand_abduction: float = 0.0,
        stand_thigh: float = 0.9,
        stand_calf: float = -1.8,
        kp_ab: float = 40.0,
        kp_thigh: float = 40.0,
        kp_calf: float = 50.0,
        kd: float = 2.0,
        wheel_radius_m: float = 0.086,
        wheel_track_m: float = 0.40,
        wheel_max_omega: float = 8.5,
        freewheel_in_legged: bool = True,
    ) -> None:
        self._env = env

        self._q_target = np.array([
            stand_abduction, stand_thigh, stand_calf,
            stand_abduction, stand_thigh, stand_calf,
            stand_abduction, stand_thigh, stand_calf,
            stand_abduction, stand_thigh, stand_calf,
        ], dtype=np.float64)

        # PD gain vectors (broadcast over 12 joints)
        self._kp = np.array([
            kp_ab, kp_thigh, kp_calf,
            kp_ab, kp_thigh, kp_calf,
            kp_ab, kp_thigh, kp_calf,
            kp_ab, kp_thigh, kp_calf,
        ], dtype=np.float64)
        self._kd = kd

        self.wheel_radius_m = wheel_radius_m
        self.wheel_track_m = wheel_track_m
        self.wheel_max_omega = wheel_max_omega
        self.freewheel_in_legged = freewheel_in_legged

        # cmd_vel from nav stack — updated by set_cmd_vel()
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._wz: float = 0.0

        # Wheel mode flag — set by hybrid router
        self._wheel_mode: bool = False

    # ── Nav interface ─────────────────────────────────────────────────────────

    def set_cmd_vel(self, vx: float, vy: float, wz: float,
                    wheel_mode: bool) -> None:
        self._vx = vx
        self._vy = vy
        self._wz = wz
        self._wheel_mode = wheel_mode

    # ── Physics step ─────────────────────────────────────────────────────────

    def step(self) -> None:
        """Compute and apply leg PD torques + wheel velocity commands.

        Standalone note: CHAMP (C++ walking gait) is not available, so both
        'wheel' and 'legged' modes drive via wheels.  The mode distinction is
        preserved for future CHAMP integration; for now both modes produce
        wheel velocity commands.  Only 'idle' (vx≈0, wz≈0) stops the wheels.
        """
        q = self._env.get_leg_qpos()
        dq = self._env.get_leg_qvel()

        err = self._q_target - q
        torques = self._kp * err - self._kd * dq
        torques = np.clip(torques, -_LIMITS, _LIMITS)
        self._env.set_leg_torques(torques)

        is_moving = (abs(self._vx) > 1e-3 or abs(self._wz) > 1e-3)
        if is_moving:
            # Both wheel_mode and legged_mode drive via wheels in standalone
            wheel_vels = self._cmd_to_wheel_vels(self._vx, self._wz)
        elif self.freewheel_in_legged:
            # Idle: mirror actual ω → zero brake torque (no wheel skid)
            wheel_vels = self._env.get_wheel_velocities()
        else:
            wheel_vels = np.zeros(4)

        self._env.set_wheel_velocities(wheel_vels)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _cmd_to_wheel_vels(self, vx: float, wz: float) -> np.ndarray:
        """Differential drive: (vx, wz) → [FL, FR, RL, RR] rad/s."""
        half = 0.5 * self.wheel_track_m
        left  = (vx - wz * half) / self.wheel_radius_m
        right = (vx + wz * half) / self.wheel_radius_m
        left  = float(np.clip(left,  -self.wheel_max_omega, self.wheel_max_omega))
        right = float(np.clip(right, -self.wheel_max_omega, self.wheel_max_omega))
        return np.array([left, right, left, right], dtype=np.float64)
