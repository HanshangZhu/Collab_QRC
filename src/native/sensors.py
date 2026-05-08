"""Native sensor reads from a live MuJoCo mjData — no ROS.

LiDARSensor: pre-computes ray directions once, fires mj_multiRay each tick.
read_pose / read_imu / read_contacts: thin wrappers over mjData arrays.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import mujoco
import numpy as np


# ---------------------------------------------------------------------------
# Dataclasses (replace ROS msgs)
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    x: float; y: float; z: float
    qw: float; qx: float; qy: float; qz: float
    t: float = 0.0


@dataclass
class ImuSample:
    accel: np.ndarray  # shape (3,)
    gyro: np.ndarray   # shape (3,)
    t: float = 0.0


@dataclass
class ContactSample:
    body1: int; body2: int
    pos: np.ndarray    # shape (3,) world frame
    normal: np.ndarray # shape (3,) world frame (from body1 to body2)
    depth: float
    t: float = 0.0


# ---------------------------------------------------------------------------
# LiDAR sensor
# ---------------------------------------------------------------------------

# Known sensor presets: (hz_samples, vt_samples, h_fov_deg, v_min_deg, v_max_deg, range_max)
_PRESETS = {
    "unitree_l1":    (360, 60, 360.0, -15.0, 15.0,  20.0),
    "livox_mid360":  (720, 16, 360.0, -7.0,  52.0,  40.0),
}


class LiDARSensor:
    """Pre-computes ray directions once at init; fires mj_multiRay each tick.

    site_name: MJCF site name (auto-detects 'livox_mid360' then 'unitree_l1').
    body_name: MJCF body to exclude from raycasting (usually 'base_link').
    returns world-frame hit points as np.ndarray[N,3] float32.
    """

    def __init__(self, model: mujoco.MjModel,
                 site_name: Optional[str] = None,
                 body_name: str = "base_link",
                 range_min: float = 0.05,
                 range_max: Optional[float] = None):

        # Auto-detect site
        if site_name is None:
            for candidate in ("livox_mid360", "unitree_l1"):
                sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, candidate)
                if sid >= 0:
                    site_name = candidate
                    break
            if site_name is None:
                raise ValueError("No known LiDAR site found in model (tried livox_mid360, unitree_l1)")

        hz, vt, h_fov, v_min, v_max, rmax_default = _PRESETS.get(site_name, (360, 16, 360.0, -15.0, 15.0, 20.0))

        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"Site '{site_name}' not found in model")

        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        self.range_min = range_min
        self.range_max = range_max if range_max is not None else rmax_default
        self.n_rays = hz * vt

        # Pre-compute ray dirs in local sensor frame (forward=+X, left=+Y, up=+Z)
        h_fov_rad = math.radians(h_fov)
        v_min_rad = math.radians(v_min)
        v_max_rad = math.radians(v_max)

        dirs = np.empty((self.n_rays, 3), dtype=np.float64)
        idx = 0
        for h in range(hz):
            h_angle = -h_fov_rad / 2.0 + h_fov_rad * h / hz
            for v in range(vt):
                v_angle = v_min_rad + (v_max_rad - v_min_rad) * v / max(vt - 1, 1)
                cos_v = math.cos(v_angle)
                dirs[idx, 0] = cos_v * math.cos(h_angle)
                dirs[idx, 1] = cos_v * math.sin(h_angle)
                dirs[idx, 2] = math.sin(v_angle)
                idx += 1
        self._dirs_local = dirs  # (N,3) local frame

        # Output buffers reused each tick
        self._geomid = np.full(self.n_rays, -1, dtype=np.int32)
        self._dist = np.full(self.n_rays, self.range_max, dtype=np.float64)

    def tick(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Returns world-frame hit points np.ndarray[M, 3] float32 (M <= n_rays)."""
        origin = data.site_xpos[self.site_id].copy()
        xmat = data.site_xmat[self.site_id].reshape(3, 3).copy()  # world_R_local

        # Rotate local dirs → world frame
        dirs_world = (self._dirs_local @ xmat.T).astype(np.float64)  # (N,3)

        self._geomid[:] = -1
        self._dist[:] = self.range_max

        mujoco.mj_multiRay(
            model, data,
            origin,
            dirs_world.ravel(),
            None,            # geomgroup: all
            1,               # flg_static
            self.body_id,    # bodyexclude
            self._geomid,
            self._dist,
            self.n_rays,
            self.range_max,
        )

        # Keep valid hits
        valid = (self._geomid >= 0) & (self._dist >= self.range_min) & (self._dist <= self.range_max)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        hit_world = origin + dirs_world[valid] * self._dist[valid, None]
        return hit_world.astype(np.float32)


# ---------------------------------------------------------------------------
# Pose, IMU, contacts
# ---------------------------------------------------------------------------

def read_pose(model: mujoco.MjModel, data: mujoco.MjData,
              body_name: str = "base_link") -> Pose:
    """Ground-truth body pose from mjData (world frame)."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"Body '{body_name}' not found")
    p = data.xpos[bid]
    q = data.xquat[bid]  # MuJoCo: w,x,y,z
    return Pose(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                qw=float(q[0]), qx=float(q[1]), qy=float(q[2]), qz=float(q[3]),
                t=float(data.time))


def _sensor_data(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sid < 0:
        raise ValueError(f"Sensor '{name}' not found")
    adr = model.sensor_adr[sid]
    dim = model.sensor_dim[sid]
    return data.sensordata[adr:adr + dim].copy()


def read_imu(model: mujoco.MjModel, data: mujoco.MjData,
             accel_name: str = "imu_accel",
             gyro_name: str = "imu_gyro") -> ImuSample:
    return ImuSample(
        accel=_sensor_data(model, data, accel_name),
        gyro=_sensor_data(model, data, gyro_name),
        t=float(data.time),
    )


def read_contacts(model: mujoco.MjModel, data: mujoco.MjData,
                  geom_set: Optional[set] = None) -> list[ContactSample]:
    """Returns active contacts. If geom_set given, only include contacts where
    both geoms belong to the set (useful for inter-robot collision detection)."""
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        if geom_set is not None:
            if c.geom1 not in geom_set and c.geom2 not in geom_set:
                continue
        b1 = model.geom_bodyid[c.geom1]
        b2 = model.geom_bodyid[c.geom2]
        normal = c.frame[:3].copy()  # first 3 elements = contact normal
        out.append(ContactSample(
            body1=int(b1), body2=int(b2),
            pos=c.pos.copy(), normal=normal,
            depth=float(c.dist),
            t=float(data.time),
        ))
    return out
