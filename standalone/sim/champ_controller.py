"""CHAMP-faithful gait controller for Go2W — standalone MuJoCo.

Ports the CHAMP C++ algorithm (Juan Miguel Jimeno, 2019-2020) from
archive/ros2_legacy/champ/champ/include/champ/ to Python.

Algorithm (4-stage pipeline, all in body frame, computed fresh each tick):
  [1] Raibert heuristic  → step_length, rotation per leg  [leg_controller.h]
  [2] Phase generator    → stance/swing phase (0→1)       [phase_generator.h]
  [3] Trajectory planner → foot target in body frame      [trajectory_planner.h]
        stance: backward sweep  x = (L/2)*(1 − 2*phase)
        swing:  12-point Bezier arc, −L/2 → +L/2
  [4] Kinematics (IK)    → joint angles                   [kinematics.h]
  [5] PD control         → joint torques

Critical design property: foot targets are computed fresh from zero_stance
(nominal stance position in body frame) every tick — no world-frame
accumulation, so IK can never diverge regardless of body drift.

Interface: identical to LegController.
    ctrl.set_cmd_vel(vx, vy, wz, wheel_mode)
    ctrl.step()
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .mujoco_env import MuJoCoEnv


# ---------------------------------------------------------------------------
# Kinematic constants (from MJCF go2w_base.xml)
# ---------------------------------------------------------------------------

_HIP_ORIGINS = np.array([
    [ 0.1934,  0.0465, 0.0],   # FL
    [ 0.1934, -0.0465, 0.0],   # FR
    [-0.1934,  0.0465, 0.0],   # RL
    [-0.1934, -0.0465, 0.0],   # RR
], dtype=np.float64)

# Lateral offset from hip joint to thigh joint (body frame Y)
_HIP_D = np.array([0.0955, -0.0955, 0.0955, -0.0955], dtype=np.float64)

_L1 = 0.213     # thigh link length (m)
_L2 = 0.2264    # calf  link length (m)
_WHEEL_R = 0.086  # wheel radius (m)

_J_LIMITS = np.array([
    [-1.0472,  1.0472,  -1.5708,  3.4907,  -2.7227, -0.83776],  # FL
    [-1.0472,  1.0472,  -1.5708,  3.4907,  -2.7227, -0.83776],  # FR
    [-1.0472,  1.0472,  -0.5236,  4.5379,  -2.7227, -0.83776],  # RL
    [-1.0472,  1.0472,  -0.5236,  4.5379,  -2.7227, -0.83776],  # RR
], dtype=np.float64)

_TAU_MAX = np.array([
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
    23.7, 23.7, 45.43,
], dtype=np.float64)

# Trot phase offsets [FL, FR, RL, RR]: diagonal pairs alternate
_PHASE_OFF = np.array([0.0, 0.5, 0.5, 0.0], dtype=np.float64)


# ---------------------------------------------------------------------------
# Derived constants (computed once at module load)
# ---------------------------------------------------------------------------

def _fk_foot_static(q_hip: float, q_thigh: float, q_calf: float, leg: int) -> np.ndarray:
    """FK for module-level constant computation (same formula as _fk_foot)."""
    hip = _HIP_ORIGINS[leg]
    d   = _HIP_D[leg]
    ch, sh = math.cos(q_hip), math.sin(q_hip)
    sx = -_L1 * math.sin(q_thigh) - _L2 * math.sin(q_thigh + q_calf)
    sz = -_L1 * math.cos(q_thigh) - _L2 * math.cos(q_thigh + q_calf)
    return (hip + np.array([sx, d * ch - sz * sh, d * sh + sz * ch])).copy()


# zero_stance[i] = nominal foot position in body frame at q_stand=[0, 0.9, -1.8]
# Equivalent to CHAMP's leg.zero_stance()
_ZERO_STANCE = np.array([_fk_foot_static(0.0, 0.9, -1.8, i) for i in range(4)])

# XY distance from body origin to zero_stance: used in Raibert turning heuristic.
# CHAMP uses lf.center_to_nominal() for all legs; we average across legs.
_CTR_TO_NOM = float(np.mean([
    math.sqrt(_ZERO_STANCE[i][0]**2 + _ZERO_STANCE[i][1]**2)
    for i in range(4)
]))


# ---------------------------------------------------------------------------
# 12-point Bezier swing trajectory (CHAMP trajectory_planner.h)
#
# Reference control points — identical to CHAMP source:
#   ref_control_points_x_: [-0.15, -0.2805, -0.3, -0.3, -0.3, 0, 0, 0, 0.3032, 0.3032, 0.2826, 0.15]
#   ref_control_points_y_: [-0.5, -0.5, -0.3611, ... -0.3214, ..., -0.5, -0.5]
#
# Scaling (per-call, based on step_length and swing_height):
#   cx[i] = ref_x[i] × (step_length / 0.4)   endpoints forced to ±step_length/2
#   cy[i] = −((ref_y[i] × h_ratio) + 0.5×h_ratio)   h_ratio = swing_height / 0.15
# ---------------------------------------------------------------------------

_BEZ_X_REF = np.array(
    [-0.15, -0.2805, -0.3, -0.3, -0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15],
    dtype=np.float64,
)
_BEZ_Y_REF = np.array(
    [-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5],
    dtype=np.float64,
)
_BEZ_I  = np.arange(12, dtype=np.float64)        # [0, 1, ..., 11]
_BEZ_NI = 11.0 - _BEZ_I                           # [11, 10, ..., 0]
_BINOM  = np.array([1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1], dtype=np.float64)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),        1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _fk_foot(q_hip: float, q_thigh: float, q_calf: float, leg: int) -> np.ndarray:
    """Forward kinematics: joint angles → foot position in base_link frame.

    Sign convention (verified vs MJCF): positive q_thigh rotates calf backward
    (−X in body frame): sx = −L1·sin(q_t) − L2·sin(q_t + q_c).
    """
    hip = _HIP_ORIGINS[leg]
    d   = _HIP_D[leg]
    ch, sh = math.cos(q_hip), math.sin(q_hip)
    sx = -_L1 * math.sin(q_thigh) - _L2 * math.sin(q_thigh + q_calf)
    sz = -_L1 * math.cos(q_thigh) - _L2 * math.cos(q_thigh + q_calf)
    return hip + np.array([sx, d * ch - sz * sh, d * sh + sz * ch])


def _ik_foot(foot_body: np.ndarray, leg: int) -> tuple[float, float, float]:
    """Analytical IK: foot position in body frame → (q_hip, q_thigh, q_calf)."""
    dp = foot_body - _HIP_ORIGINS[leg]
    d  = _HIP_D[leg]

    # Hip abduction
    r_yz = math.sqrt(dp[1]**2 + dp[2]**2)
    if r_yz > abs(d) + 1e-6:
        l_sag = math.sqrt(max(0.0, r_yz**2 - d**2))
        gamma = math.atan2(dp[1], -dp[2])
        delta = math.atan2(d, l_sag)
        q_hip = gamma - delta
    else:
        l_sag = 0.0
        q_hip = 0.0

    # 2-link planar IK in sagittal plane
    l_sag = math.sqrt(max(0.0, r_yz**2 - d**2))
    p2x, p2z = dp[0], -l_sag
    L = math.sqrt(p2x**2 + p2z**2)
    L = max(1e-3, min(L, _L1 + _L2 - 1e-4))

    cos_c   = (L**2 - _L1**2 - _L2**2) / (2.0 * _L1 * _L2)
    q_calf  = -math.acos(max(-1.0, min(1.0, cos_c)))

    alpha   = math.atan2(-p2x, -p2z)
    cos_t   = (L**2 + _L1**2 - _L2**2) / (2.0 * L * _L1)
    beta    = math.acos(max(-1.0, min(1.0, cos_t)))
    q_thigh = alpha + beta

    lim = _J_LIMITS[leg]
    q_hip   = max(lim[0], min(lim[1], q_hip))
    q_thigh = max(lim[2], min(lim[3], q_thigh))
    q_calf  = max(lim[4], min(lim[5], q_calf))
    return float(q_hip), float(q_thigh), float(q_calf)


def _bezier_swing(t: float, step_length: float, swing_height: float) -> tuple[float, float]:
    """12-point Bezier swing arc — matches CHAMP trajectory_planner.h exactly.

    Returns (Δx, Δz) foot displacement relative to zero_stance:
      Δx: along step direction (−L/2 at t=0 liftoff, +L/2 at t=1 touchdown)
      Δz: vertical lift (0 at endpoints, ≈swing_height at t≈0.5)
    """
    len_ratio = step_length / 0.4
    hgt_ratio = swing_height / 0.15

    cx = _BEZ_X_REF * len_ratio
    cx = cx.copy()
    cx[0]  = -step_length / 2.0    # force exact endpoints
    cx[11] =  step_length / 2.0

    # CHAMP: control_points_y[i] = -((ref_y[i] * h_ratio) + 0.5 * h_ratio)
    cy = -((_BEZ_Y_REF * hgt_ratio) + 0.5 * hgt_ratio)

    t_c     = max(0.0, min(1.0, t))
    t_pows  = t_c ** _BEZ_I
    tm_pows = (1.0 - t_c) ** _BEZ_NI
    b       = _BINOM * t_pows * tm_pows

    x = float(b @ cx)
    y = float(-(b @ cy))   # CHAMP: y -= b·cy[i]  →  y = −(b·cy)
    return x, y


# ---------------------------------------------------------------------------
# ChampParams
# ---------------------------------------------------------------------------

@dataclass
class ChampParams:
    # Gait timing (matches CHAMP gait.yaml conventions)
    gait_period:   float = 0.50   # s — full trot cycle
    duty_factor:   float = 0.50   # fraction in stance (0.5 = trot)

    # Swing trajectory
    swing_height:  float = 0.08   # m — peak foot lift (CHAMP: swing_height)
    stance_depth:  float = 0.00   # m — cosine ground depression in stance

    # Body height target (used by warm-start; passive IK regulates height)
    body_height:   float = 0.36   # m

    # Joint PD (settle phase and trot PD)
    kp_leg: float = 60.0
    kd_leg: float = 3.0

    # Wheel actuator
    wheel_radius_m:  float = 0.086
    wheel_track_m:   float = 0.40
    wheel_max_omega: float = 8.5

    # Speed threshold above which wheel mode is preferred over trot
    wheel_mode_speed: float = 1.0  # m/s

    @classmethod
    def from_dict(cls, d: dict) -> "ChampParams":
        fields = {f.name for f in cls.__dataclass_fields__.values()}
        return cls(**{k: v for k, v in d.items() if k in fields})


# ---------------------------------------------------------------------------
# ChampController
# ---------------------------------------------------------------------------

class ChampController:
    """CHAMP-faithful gait controller for Go2W.

    Drop-in replacement for LegController. Implements the 4-stage CHAMP
    pipeline (Raibert → phase → trajectory → IK → PD) entirely in body frame.

    Body height is regulated passively: stance feet are always targeted at
    zero_stance[z] ≈ WHEEL_R − body_height in body frame. If the body dips,
    the PD drives legs to extend, which pushes the body back up via ground
    reaction forces.
    """

    def __init__(self, env: MuJoCoEnv, params: ChampParams | None = None) -> None:
        self._env = env
        self._p   = params or ChampParams()
        self._dt  = env.timestep

        # Derived timing
        self._stance_duration = self._p.gait_period * self._p.duty_factor

        # cmd_vel from nav stack
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._wz: float = 0.0
        self._wheel_mode: bool = False

        # Gait phase (0→1 over one gait_period)
        self._global_phase: float = 0.0

        # Settle config
        self._settle_steps = int(1.5 / self._dt)   # 1.5 s settle
        self._step_count   = 0
        self._kp_settle    = np.array([40.0, 40.0, 50.0] * 4, dtype=np.float64)
        self._kd_settle    = 2.0
        self._q_stand      = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)

        # Warm-start: pre-set qpos so settle starts within a few degrees of
        # q_stand. Without this, the settle PD applies ~36 Nm backward torque
        # on the thigh (error 0→0.9 rad) and pitches the robot nose-down.
        for i, addr in enumerate(env._leg_qpos_addrs):
            if addr >= 0:
                env.data.qpos[addr] = self._q_stand[i]
        env.data.qpos[2] = self._p.body_height
        env.forward()

    # ── Public interface ─────────────────────────────────────────────────────

    def set_cmd_vel(self, vx: float, vy: float, wz: float,
                    wheel_mode: bool) -> None:
        self._vx = vx
        self._vy = vy
        self._wz = wz
        self._wheel_mode = wheel_mode

    # ── Main tick ────────────────────────────────────────────────────────────

    def step(self) -> None:
        self._step_count += 1

        if self._step_count <= self._settle_steps:
            self._settle_step()
            return

        use_wheels = (
            self._wheel_mode
            or abs(self._vx) >= self._p.wheel_mode_speed
        )
        if use_wheels:
            self._wheel_drive_step()
        else:
            self._trot_step()

    # ── Settle (standing pose PD) ────────────────────────────────────────────

    def _settle_step(self) -> None:
        q   = self._env.get_leg_qpos()
        dq  = self._env.get_leg_qvel()
        tau = self._kp_settle * (self._q_stand - q) - self._kd_settle * dq
        tau = np.clip(tau, -_TAU_MAX, _TAU_MAX)
        self._env.set_leg_torques(tau)
        self._env.set_wheel_velocities(self._env.get_wheel_velocities())

    # ── Wheel-drive mode ─────────────────────────────────────────────────────

    def _wheel_drive_step(self) -> None:
        # Hold legs at q_stand (IK inverse of zero_stance) while wheels drive.
        self._apply_pd(self._q_stand)

        is_moving = abs(self._vx) > 1e-3 or abs(self._wz) > 1e-3
        if is_moving:
            self._env.set_wheel_velocities(self._cmd_to_wheel(self._vx, self._wz))
        else:
            self._env.set_wheel_velocities(self._env.get_wheel_velocities())

    # ── Trot gait (CHAMP body-frame algorithm) ───────────────────────────────

    def _trot_step(self) -> None:
        is_moving = abs(self._vx) > 0.02 or abs(self._vy) > 0.02 or abs(self._wz) > 0.02

        if is_moving:
            self._global_phase = (
                self._global_phase + self._dt / self._p.gait_period
            ) % 1.0

        q_target = np.zeros(12, dtype=np.float64)

        for i in range(4):
            leg_phase = (self._global_phase + _PHASE_OFF[i]) % 1.0
            in_swing  = is_moving and (leg_phase >= self._p.duty_factor)

            step_length, rotation = self._step_params(i)
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)

            if in_swing:
                # Swing: 12-point Bezier arc from −L/2 to +L/2 above zero_stance
                sp = (leg_phase - self._p.duty_factor) / max(
                    1e-6, 1.0 - self._p.duty_factor
                )
                sx, sz = _bezier_swing(sp, step_length, self._p.swing_height)
            else:
                # Stance: backward sweep under the body
                tp = leg_phase / max(1e-6, self._p.duty_factor)
                sx = (step_length / 2.0) * (1.0 - 2.0 * tp)
                if step_length > 1e-4:
                    sz = -self._p.stance_depth * math.cos(
                        math.pi * sx / step_length
                    )
                else:
                    sz = 0.0

            foot_b = _ZERO_STANCE[i] + np.array([sx * cos_r, sx * sin_r, sz])
            q_hip, q_thigh, q_calf = _ik_foot(foot_b, i)
            q_target[3*i : 3*i+3] = [q_hip, q_thigh, q_calf]

        self._apply_pd(q_target)

        # Wheel velocity: freewheel during trot — mirror the previous tick's ω.
        #
        # In stance the wheel rolls at ω = v_body / r.  In swing (no ground
        # contact) ctrl persists at that value, so the wheel keeps spinning at
        # the correct speed.  On touchdown: ctrl ≈ ω_actual → τ_brake ≈ 0.
        #
        # Why NOT _cmd_to_wheel(vx_commanded)?
        #   vx_cmd = 0.2 m/s → ctrl = 2.33 rad/s.  Actual trot body speed is
        #   ~0.024 m/s → ω_roll ≈ 0.28 rad/s.  Drive torque = kv×(2.33−0.28)
        #   = 10 Nm per wheel — active grinding against the floor → 88% slip.
        self._env.set_wheel_velocities(self._env.get_wheel_velocities())

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _step_params(self, leg: int) -> tuple[float, float]:
        """Raibert heuristic + transformLeg → (step_length, rotation).

        Matches CHAMP leg_controller.h: raibertHeuristic() + transformLeg().
        step_length is the full ground-contact arc the foot sweeps per cycle.
        rotation is the direction of the sweep in the body XY plane.
        """
        step_x = self._stance_duration * 0.5 * self._vx
        step_y = self._stance_duration * 0.5 * self._vy

        # Turning: tangential velocity at the nominal foot radius
        tangential = self._wz * _CTR_TO_NOM
        step_th_dist = self._stance_duration * 0.5 * tangential
        sin_val = max(-1.0, min(1.0, step_th_dist / (2.0 * _CTR_TO_NOM)))
        theta = 2.0 * math.asin(sin_val)

        # transformLeg: translate zero_stance by (step_x, step_y), rotate by theta
        zs = _ZERO_STANCE[leg]
        tx = zs[0] + step_x
        ty = zs[1] + step_y
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        rx = tx * cos_t - ty * sin_t
        ry = tx * sin_t + ty * cos_t

        delta_x = rx - zs[0]
        delta_y = ry - zs[1]
        step_length = math.sqrt(delta_x**2 + delta_y**2) * 2.0
        step_length = max(step_length, 1e-4)
        rotation    = math.atan2(delta_y, delta_x)
        return step_length, rotation

    def _apply_pd(self, q_target: np.ndarray) -> None:
        """Joint-space PD → torques."""
        q   = self._env.get_leg_qpos()
        dq  = self._env.get_leg_qvel()
        tau = self._p.kp_leg * (q_target - q) - self._p.kd_leg * dq
        tau = np.clip(tau, -_TAU_MAX, _TAU_MAX)
        self._env.set_leg_torques(tau)

    def _cmd_to_wheel(self, vx: float, wz: float) -> np.ndarray:
        """Differential drive: (vx, wz) → [FL, FR, RL, RR] rad/s."""
        half  = 0.5 * self._p.wheel_track_m
        r     = self._p.wheel_radius_m
        left  = float(np.clip(
            (vx - wz * half) / r, -self._p.wheel_max_omega, self._p.wheel_max_omega
        ))
        right = float(np.clip(
            (vx + wz * half) / r, -self._p.wheel_max_omega, self._p.wheel_max_omega
        ))
        return np.array([left, right, left, right], dtype=np.float64)
