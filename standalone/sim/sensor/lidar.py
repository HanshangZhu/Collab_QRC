"""MuJoCo LiDAR raycast sensor — ROS2-free version of mujoco_lidar_node.py.

Casts rays directly against the live MuJoCo model+data owned by MuJoCoEnv.
Returns world-frame 3D points as a (N, 3) float32 array.

Ray directions are pre-computed once at construction; at each scan the lidar
site pose is read from data.site_xpos/xmat and the rays are rotated to world
frame before calling mj_ray().
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from ..mujoco_env import MuJoCoEnv


class Lidar:
    """Livox MID-360 style LiDAR raycast from the MuJoCo sim.

    Parameters match the existing mujoco_lidar_node defaults so behaviour is
    identical to the ROS2 version (just fewer rays for performance, tunable).
    """

    def __init__(
        self,
        env: MuJoCoEnv,
        *,
        hz_samples: int = 360,
        vt_samples: int = 8,
        h_fov_deg: float = 360.0,
        v_min_deg: float = -7.0,
        v_max_deg: float = 52.0,
        range_min: float = 0.05,
        range_max: float = 8.0,
    ) -> None:
        self._env = env
        self._range_min = range_min
        self._range_max = range_max

        # Pre-compute ray directions in LiDAR-local frame (x-fwd, y-left, z-up)
        h_fov = math.radians(h_fov_deg)
        v_min = math.radians(v_min_deg)
        v_max = math.radians(v_max_deg)

        h_angles = np.linspace(-h_fov / 2, h_fov / 2, hz_samples, endpoint=False)
        v_angles = np.linspace(v_min, v_max, vt_samples)
        h_grid, v_grid = np.meshgrid(h_angles, v_angles, indexing="ij")
        h_flat = h_grid.ravel()
        v_flat = v_grid.ravel()
        cos_v = np.cos(v_flat)
        self._ray_dirs_local = np.stack([
            cos_v * np.cos(h_flat),
            cos_v * np.sin(h_flat),
            np.sin(v_flat),
        ], axis=1).astype(np.float64)  # (N, 3)
        self._n_rays = len(h_flat)

        # geomgroup = None means cast against all geoms
        self._geomgroup: None = None

    # ── Scan ─────────────────────────────────────────────────────────────────

    def scan(self) -> np.ndarray:
        """Return world-frame hit points as (M, 3) float32 array.

        M ≤ N where N = hz_samples × vt_samples.
        Points are in WORLD frame (absolute coordinates).
        """
        env = self._env
        origin = env.get_lidar_origin()          # (3,) world pos of lidar site
        mat = env.get_lidar_mat()                # (3,3) rotation matrix

        # Guard: skip if site matrix is degenerate (can happen at t=0 before
        # the first physics step populates site_xmat).
        if not np.isfinite(mat).all() or abs(np.linalg.det(mat)) < 1e-6:
            return np.empty((0, 3), dtype=np.float32)

        # Rotate local ray dirs to world frame
        ray_dirs_world = (mat @ self._ray_dirs_local.T).T  # (N, 3)

        hit_points = np.empty((self._n_rays, 3), dtype=np.float32)
        valid = np.zeros(self._n_rays, dtype=bool)
        geomid = np.array([-1], dtype=np.int32)

        model = env.model
        data = env.data
        body_exclude = env.base_body_id

        for i in range(self._n_rays):
            dist = mujoco.mj_ray(
                model, data,
                origin,
                ray_dirs_world[i],
                self._geomgroup,
                1,             # flg_static: include static geoms
                body_exclude,  # exclude robot body
                geomid,
            )
            if self._range_min <= dist <= self._range_max:
                hit_points[i] = (origin + ray_dirs_world[i] * dist).astype(np.float32)
                valid[i] = True

        return hit_points[valid]
