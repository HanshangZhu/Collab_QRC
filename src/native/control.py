"""Go2W wheel-direct controller — no ROS, no CHAMP.

Stance: leg PD keeps fixed joint angles (Go2W standing pose).
Motion: diff-drive kinematics → 4 wheel velocity setpoints in ctrl[].
"""
from __future__ import annotations

import mujoco
import numpy as np

# Go2W standing stance (rad): hip=0, thigh=0.9, calf=-1.8
_STANCE = {
    "FL_hip": 0.0, "FL_thigh": 0.9, "FL_calf": -1.8,
    "FR_hip": 0.0, "FR_thigh": 0.9, "FR_calf": -1.8,
    "RL_hip": 0.0, "RL_thigh": 0.9, "RL_calf": -1.8,
    "RR_hip": 0.0, "RR_thigh": 0.9, "RR_calf": -1.8,
}

_LEG_JOINTS = list(_STANCE.keys())
_WHEEL_ACTUATORS = ["FL_wheel", "FR_wheel", "RL_wheel", "RR_wheel"]

WHEEL_RADIUS = 0.05   # m (cylinder geom size[0] in go2w_base.xml)
TRACK_WIDTH  = 0.284  # m (2 * (hip_y + thigh_y offset) ≈ 2 * 0.142)

KP = 40.0  # N·m/rad
KD = 2.0   # N·m·s/rad


class WheelDirectController:
    """Pre-resolves all MuJoCo IDs at init; applies PD stance + wheel vel each tick."""

    def __init__(self, model: mujoco.MjModel,
                 kp: float = KP, kd: float = KD,
                 wheel_radius: float = WHEEL_RADIUS,
                 track_width: float = TRACK_WIDTH):
        self.kp = kp
        self.kd = kd
        self.wheel_radius = wheel_radius
        self.half_track = track_width / 2.0

        # Resolve leg joint → (qpos_adr, dof_adr, target_angle, ctrl_idx)
        # Actuator names: FL_hip, FL_thigh, FL_calf, ...
        # Joint names in MJCF: FL_hip_joint, FL_thigh_joint, FL_calf_joint, ...
        self._leg_info: list[tuple[int, int, float, int]] = []
        for aname in _LEG_JOINTS:
            jname = aname + "_joint"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"Joint '{jname}' not found")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                raise ValueError(f"Actuator '{aname}' not found")
            self._leg_info.append((
                model.jnt_qposadr[jid],
                model.jnt_dofadr[jid],
                _STANCE[aname],
                aid,
            ))

        # Resolve wheel actuator ctrl indices
        self._wheel_aids: list[int] = []
        for wname in _WHEEL_ACTUATORS:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, wname)
            if aid < 0:
                raise ValueError(f"Actuator '{wname}' not found")
            self._wheel_aids.append(aid)
        # wheel order: FL(L), FR(R), RL(L), RR(R)
        # left = indices 0,2; right = indices 1,3

        self._vx = 0.0
        self._wz = 0.0

    def set_cmd_vel(self, vx: float, wz: float) -> None:
        self._vx = vx
        self._wz = wz

    def tick(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Apply one control tick. Call once per mj_step."""
        # Leg PD
        for q_adr, dof_adr, q_des, aid in self._leg_info:
            q = data.qpos[q_adr]
            qd = data.qvel[dof_adr]
            tau = self.kp * (q_des - q) + self.kd * (0.0 - qd)
            data.ctrl[aid] = float(np.clip(tau, model.actuator_ctrlrange[aid, 0],
                                           model.actuator_ctrlrange[aid, 1]))

        # Diff-drive: left side = FL(0), RL(2); right side = FR(1), RR(3)
        v_left  = (self._vx - self._wz * self.half_track) / self.wheel_radius
        v_right = (self._vx + self._wz * self.half_track) / self.wheel_radius

        for i, aid in enumerate(self._wheel_aids):
            v = v_left if i % 2 == 0 else v_right
            data.ctrl[aid] = float(np.clip(v, model.actuator_ctrlrange[aid, 0],
                                           model.actuator_ctrlrange[aid, 1]))
