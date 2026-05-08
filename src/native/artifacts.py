"""Dynamic artifact detection — no hardcoded positions.

At init: scans MjModel geoms, filters out walls/ground/floors by size+type+name.
At runtime: checks robot proximity to each candidate geom using live mjData.

Works in any MJCF scene. No scene-specific knowledge required.

Filter logic (configurable):
  - Exclude geom types: plane (ground)
  - Exclude geoms whose name matches any exclude_name_substr (walls, dividers, etc.)
  - Exclude geoms with largest dimension > max_size_m (walls, large obstacles)
  - Exclude geoms with largest dimension < min_size_m (tiny collision geometry)
  - What remains = "artifact" candidates

Returns world (x, y, z) coordinates from live mjData.geom_xpos — works for
both static world-body geoms and dynamic geoms on moving bodies.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import mujoco
import numpy as np


_DEFAULT_EXCLUDE = (
    "wall", "divider", "floor", "ground", "plane",
    "ceiling", "collision", "foot", "hip", "thigh",
    "calf", "base", "wheel", "sensor", "lidar",
)


@dataclass
class Artifact:
    name: str
    x: float
    y: float
    z: float
    geom_id: int
    found: bool = False
    found_at_t: float = 0.0
    found_dist: float = 0.0


class ArtifactDetector:
    """Discovers artifact geom candidates from the model; detects them at runtime."""

    def __init__(self,
                 model: mujoco.MjModel,
                 detect_dist: float = 2.0,
                 min_size_m: float = 0.03,
                 max_size_m: float = 0.80,
                 exclude_name_substr: tuple[str, ...] = _DEFAULT_EXCLUDE):
        self.detect_dist = detect_dist
        self._artifacts: list[Artifact] = []

        for gid in range(model.ngeom):
            # Skip planes (ground)
            if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_PLANE:
                continue

            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom_{gid}"

            # Skip by name
            name_lower = name.lower()
            if any(ex in name_lower for ex in exclude_name_substr):
                continue

            # Filter by size: model.geom_size[gid] stores (r, half-length, 0) for cylinder
            # and (hx, hy, hz) for box, (r, 0, 0) for sphere
            sizes = model.geom_size[gid]
            largest = float(np.max(sizes[sizes > 0])) if np.any(sizes > 0) else 0.0
            if largest < min_size_m or largest > max_size_m:
                continue

            # Placeholder position (will be updated from mjData.geom_xpos at runtime)
            pos = model.geom_pos[gid]  # local frame — use geom_xpos at runtime
            self._artifacts.append(Artifact(
                name=name,
                x=float(pos[0]),
                y=float(pos[1]),
                z=float(pos[2]),
                geom_id=gid,
            ))

        print(f"ArtifactDetector: found {len(self._artifacts)} candidates: "
              f"{[a.name for a in self._artifacts]}")

    # ── Runtime ──────────────────────────────────────────────────────────────

    def tick(self, data: mujoco.MjData, robot_x: float, robot_y: float,
             sim_time: float = 0.0) -> list[Artifact]:
        """Check proximity. Returns list of newly found artifacts this tick."""
        newly_found: list[Artifact] = []
        for a in self._artifacts:
            if a.found:
                continue
            # Live world-frame geom position
            pos = data.geom_xpos[a.geom_id]
            a.x, a.y, a.z = float(pos[0]), float(pos[1]), float(pos[2])
            dist = math.hypot(robot_x - a.x, robot_y - a.y)
            if dist <= self.detect_dist:
                a.found = True
                a.found_at_t = sim_time
                a.found_dist = dist
                newly_found.append(a)
        return newly_found

    # ── Accessors ─────────────────────────────────────────────────────────────

    @property
    def all_artifacts(self) -> list[Artifact]:
        return list(self._artifacts)

    @property
    def found(self) -> list[Artifact]:
        return [a for a in self._artifacts if a.found]

    @property
    def unfound(self) -> list[Artifact]:
        return [a for a in self._artifacts if not a.found]

    def summary(self) -> str:
        lines = [f"Artifacts found: {len(self.found)}/{len(self._artifacts)}"]
        for a in self._artifacts:
            status = f"FOUND at t={a.found_at_t:.0f}s dist={a.found_dist:.2f}m" if a.found else "not found"
            lines.append(f"  {a.name:30s}  ({a.x:6.2f}, {a.y:6.2f})  {status}")
        return "\n".join(lines)
