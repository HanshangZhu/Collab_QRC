#!/usr/bin/env python3
"""Standalone MuJoCo LiDAR raycast test — no ROS 2 required.

Loads the VLM exploration scene, steps physics to let the robot settle,
then casts rays from the LiDAR mount and visualises the resulting pointcloud.
"""

import math
import time

import mujoco
import mujoco.viewer
import numpy as np

# ── Config ──────────────────────────────────────────────────────────
MJCF_PATH = "src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml"
LIDAR_BODY = "base_link"
LIDAR_OFFSET = np.array([0.16143, 0.0, 0.12262], dtype=np.float64)

HZ_SAMPLES = 720
VT_SAMPLES = 16
H_FOV_DEG = 360.0
V_MIN_DEG = -7.0
V_MAX_DEG = 52.0
RANGE_MIN = 0.05
RANGE_MAX = 20.0

SETTLE_SECONDS = 1.0  # let robot drop onto ground


def build_ray_dirs():
    """Pre-compute (N,3) local-frame ray directions."""
    h_fov = math.radians(H_FOV_DEG)
    v_min = math.radians(V_MIN_DEG)
    v_max = math.radians(V_MAX_DEG)

    h_angles = np.linspace(-h_fov / 2, h_fov / 2, HZ_SAMPLES, endpoint=False)
    v_angles = np.linspace(v_min, v_max, VT_SAMPLES)

    h_grid, v_grid = np.meshgrid(h_angles, v_angles, indexing="ij")
    h_flat, v_flat = h_grid.ravel(), v_grid.ravel()

    cos_v = np.cos(v_flat)
    dirs = np.stack([
        cos_v * np.cos(h_flat),
        cos_v * np.sin(h_flat),
        np.sin(v_flat),
    ], axis=1).astype(np.float64)
    return dirs


def cast_rays(model, data, body_id, ray_dirs_local):
    """Cast rays from LiDAR mount, return (M,3) hit points in world frame."""
    body_pos = data.xpos[body_id].copy()
    body_mat = data.xmat[body_id].reshape(3, 3).copy()

    origin = body_pos + body_mat @ LIDAR_OFFSET
    dirs_world = (body_mat @ ray_dirs_local.T).T

    n = len(dirs_world)
    hits = np.empty((n, 3), dtype=np.float64)
    valid = np.zeros(n, dtype=bool)
    geomid_out = np.array([-1], dtype=np.int32)

    for i in range(n):
        dist = mujoco.mj_ray(
            model, data, origin, dirs_world[i],
            None,       # all geom groups
            1,          # include static
            body_id,    # exclude robot
            geomid_out,
        )
        if RANGE_MIN <= dist <= RANGE_MAX:
            hits[i] = origin + dirs_world[i] * dist
            valid[i] = True

    return hits[valid], origin


def main():
    print(f"Loading model: {MJCF_PATH}")
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LIDAR_BODY)
    assert body_id >= 0, f"Body '{LIDAR_BODY}' not found"

    # Settle physics
    n_steps = int(SETTLE_SECONDS / model.opt.timestep)
    print(f"Settling physics for {SETTLE_SECONDS}s ({n_steps} steps) ...")
    for _ in range(n_steps):
        mujoco.mj_step(model, data)

    mujoco.mj_forward(model, data)

    ray_dirs = build_ray_dirs()
    print(f"Casting {len(ray_dirs)} rays ...")
    t0 = time.perf_counter()
    points, origin = cast_rays(model, data, body_id, ray_dirs)
    dt = time.perf_counter() - t0
    print(f"Got {len(points)} hits in {dt*1000:.1f} ms")
    print(f"LiDAR origin (world): {origin}")

    if len(points) == 0:
        print("No hits — check your scene geometry.")
        return

    # Stats
    dists = np.linalg.norm(points - origin, axis=1)
    print(f"Range: {dists.min():.2f} – {dists.max():.2f} m, mean {dists.mean():.2f} m")
    print(f"Bounding box: x=[{points[:,0].min():.2f}, {points[:,0].max():.2f}]  "
          f"y=[{points[:,1].min():.2f}, {points[:,1].max():.2f}]  "
          f"z=[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

    # ── Visualize ──
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        fig = plt.figure(figsize=(14, 6))

        # Top-down (X-Y)
        ax1 = fig.add_subplot(121)
        ax1.scatter(points[:, 0], points[:, 1], s=0.2, c=points[:, 2], cmap="viridis")
        ax1.plot(origin[0], origin[1], "r+", markersize=12, markeredgewidth=2)
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title("Top-down view (colored by Z)")
        ax1.set_aspect("equal")
        ax1.grid(True, alpha=0.3)

        # 3D
        ax2 = fig.add_subplot(122, projection="3d")
        ax2.scatter(points[:, 0], points[:, 1], points[:, 2], s=0.3, c=points[:, 2], cmap="viridis")
        ax2.scatter(*origin, c="red", s=50, marker="^", label="LiDAR")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.set_zlabel("Z")
        ax2.set_title("3D pointcloud")
        ax2.legend()

        plt.tight_layout()
        plt.savefig("/tmp/mujoco_lidar_test.png", dpi=150)
        print("Saved plot to /tmp/mujoco_lidar_test.png")
        plt.show()
    except ImportError:
        print("matplotlib not available — skipping visualization")


if __name__ == "__main__":
    main()
