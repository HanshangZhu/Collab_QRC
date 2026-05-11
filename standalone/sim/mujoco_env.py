"""MuJoCo environment wrapper — loads MJCF, steps physics, exposes GT state."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import mujoco
import numpy as np


# Actuator indices in demo1.xml (order they appear in <actuator>)
_LEG_CTRL = slice(0, 12)   # 12 torque motors: [FL_hip, FL_thigh, FL_calf, FR_, RL_, RR_]
_WHEEL_CTRL = slice(12, 16) # 4 velocity actuators: [FL, FR, RL, RR] wheel

# Sensor indices
_SENS_BASE_POS  = slice(10, 13)  # framepos  base_link_site  (x, y, z)
_SENS_BASE_QUAT = slice(13, 17)  # framequat base_link_site  (w, x, y, z)
_SENS_BASE_LINVEL = slice(17, 20)
_SENS_BASE_ANGVEL = slice(20, 23)

Pose2D = Tuple[float, float, float]  # (x, y, yaw)


class MuJoCoEnv:
    """Thin wrapper around a MuJoCo model + data pair.

    Owns the physics step, GT pose readout, and actuator write.
    Does NOT own the viewer — that lives in main.py so it can be optional.
    """

    def __init__(self, mjcf_path: str | Path) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        # Cache body/site ids
        self._base_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "base_link_site")
        self._lidar_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "livox_mid360")
        self._base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

        # Cache wheel joint qvel addresses for freewheel readout
        self._wheel_joint_names = [
            "FL_foot_joint", "FR_foot_joint",
            "RL_foot_joint", "RR_foot_joint",
        ]
        self._wheel_qvel_addrs: list[int] = []
        for jname in self._wheel_joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                self._wheel_qvel_addrs.append(int(self.model.jnt_dofadr[jid]))
            else:
                self._wheel_qvel_addrs.append(-1)

        # Cache leg joint qpos/qvel addresses
        _leg_joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        ]
        self._leg_qpos_addrs: list[int] = []
        self._leg_qvel_addrs: list[int] = []
        for jname in _leg_joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                self._leg_qpos_addrs.append(int(self.model.jnt_qposadr[jid]))
                self._leg_qvel_addrs.append(int(self.model.jnt_dofadr[jid]))
            else:
                self._leg_qpos_addrs.append(-1)
                self._leg_qvel_addrs.append(-1)

    # ── Physics ──────────────────────────────────────────────────────────────

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    # ── GT pose ──────────────────────────────────────────────────────────────

    def get_pose3d(self) -> np.ndarray:
        """Return (x, y, z) from sensor framepos."""
        return self.data.sensordata[_SENS_BASE_POS].copy()

    def get_quat(self) -> np.ndarray:
        """Return (w, x, y, z) from sensor framequat."""
        return self.data.sensordata[_SENS_BASE_QUAT].copy()

    def get_pose2d(self) -> Pose2D:
        """Return (x, y, yaw) — yaw extracted from quaternion."""
        pos = self.get_pose3d()
        q = self.get_quat()  # w, x, y, z
        yaw = _quat_to_yaw(q)
        return (float(pos[0]), float(pos[1]), float(yaw))

    def get_linvel(self) -> np.ndarray:
        return self.data.sensordata[_SENS_BASE_LINVEL].copy()

    # ── Site pose (for lidar) ─────────────────────────────────────────────────

    def get_lidar_origin(self) -> np.ndarray:
        return self.data.site_xpos[self._lidar_site_id].copy()

    def get_lidar_mat(self) -> np.ndarray:
        return self.data.site_xmat[self._lidar_site_id].reshape(3, 3).copy()

    # ── Actuator write ───────────────────────────────────────────────────────

    def set_leg_torques(self, torques: np.ndarray) -> None:
        """Write 12 leg joint torques (indices 0-11)."""
        self.data.ctrl[_LEG_CTRL] = torques

    def set_wheel_velocities(self, vels: np.ndarray) -> None:
        """Write 4 wheel velocity setpoints (indices 12-15)."""
        self.data.ctrl[_WHEEL_CTRL] = vels

    # ── Joint state readout ──────────────────────────────────────────────────

    def get_leg_qpos(self) -> np.ndarray:
        out = np.zeros(12)
        for i, addr in enumerate(self._leg_qpos_addrs):
            if addr >= 0:
                out[i] = self.data.qpos[addr]
        return out

    def get_leg_qvel(self) -> np.ndarray:
        out = np.zeros(12)
        for i, addr in enumerate(self._leg_qvel_addrs):
            if addr >= 0:
                out[i] = self.data.qvel[addr]
        return out

    def get_wheel_velocities(self) -> np.ndarray:
        """Read actual wheel angular velocities (rad/s) from qvel."""
        out = np.zeros(4)
        for i, addr in enumerate(self._wheel_qvel_addrs):
            if addr >= 0:
                out[i] = self.data.qvel[addr]
        return out

    # ── Helpers ──────────────────────────────────────────────────────────────

    @property
    def base_body_id(self) -> int:
        return self._base_body_id

    @property
    def lidar_site_id(self) -> int:
        return self._lidar_site_id


def _quat_to_yaw(q: np.ndarray) -> float:
    """Extract yaw from quaternion (w, x, y, z)."""
    w, x, y, z = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
