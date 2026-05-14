# Standalone CHAMP Stack — Development Log (2026-05)

Full history of what was built, every bug hit, every fix applied, and open issues as of 2026-05-14.  
Algorithm and kinematic reference: [`champ_standalone.md`](champ_standalone.md).

---

## What was built

A **standalone MuJoCo exploration stack** that runs entirely without ROS 2. The stack mirrors the full production pipeline (LiDAR → occupancy map → CFPA2 frontier planner → A* path → controller → robot) but runs as a single Python process against a MuJoCo physics scene.

Two entry points:

| Script | Controller | Locomotion |
|---|---|---|
| `standalone/main.py` | `LegController` | Wheels only — legs locked at `q_stand` |
| `standalone/main_champ.py` | `ChampController` | Full trot gait (CHAMP body-frame algorithm) |

Key modules added / modified during this session:

| File | What changed |
|---|---|
| `standalone/sim/champ_controller.py` | New file — full CHAMP port (Raibert + Bezier + IK + PD) |
| `standalone/sim/mujoco_env.py` | Added `get_foot_positions()`, `get_pose3d()` |
| `standalone/mapping/occupancy_grid.py` | `update()` now accepts `robot_z` kwarg (floor-hit fix) |
| `standalone/main_champ.py` | New file — exploration loop wired to `ChampController` + `MapViz` |
| `standalone/config.yaml` | Added `champ:` block, removed unused `kp_body`/`kd_body` |
| `docs/claude/champ_standalone.md` | Algorithm reference doc |

---

## Session timeline

### 1 — CHAMP controller initial port

**Goal:** Implement the 4-stage CHAMP pipeline in Python, coexisting with `LegController`.

**Decisions:**
- Separate entry point `main_champ.py` (not patching `main.py`) so both can be benchmarked side-by-side.
- Fidelity target: identical Bezier control points, identical Raibert formula, identical body-frame approach as the C++ source (`archive/ros2_legacy/champ/champ/include/champ/`).
- Full port (not wrapping the compiled C++ via ROS 2) because the standalone stack has no ROS 2 bridge, no `joint_group_position_controller`, and writes torques directly to MuJoCo `data.ctrl` at 500 Hz.

---

### 2 — Bug: robot flips immediately on spawn

**Symptom:** Robot body hits the floor within 0.5 s of launch.

**Root cause:** MJCF spawns at `body_z = 0.6 m` with all joints `q = 0` (legs fully extended, feet ~16 cm above floor). The settle PD applied maximum backward thigh torque (`error = 0 → 0.9 rad`) before the legs could reach `q_stand`, pitching the nose down → flip.

**Fix:** Warm-start in `ChampController.__init__` — pre-set leg `qpos` to `q_stand` and body `z` to `body_height` before any physics step:

```python
for i, addr in enumerate(env._leg_qpos_addrs):
    env.data.qpos[addr] = self._q_stand[i]
env.data.qpos[2] = self._p.body_height
env.forward()
```

---

### 3 — Bug: robot flips mid-trot (~45 s in)

**Symptom:** Robot navigated correctly for ~45 s (3 path assignments) then flipped.

**Initial hypothesis:** spawn handling — ruled out (robot was stable for 45 s).

**Root cause: world-frame stance foot freeze.** The original implementation stored stance foot positions in world frame and held them fixed while the robot moved. As the body advanced, the frozen world-frame foot drifted *backward* in body frame:

| Tick | Stance foot (world) | Body x | Foot in body frame |
|---|---|---|---|
| 0 | 4.20 m | 4.00 m | +0.20 m (nominal) |
| N | 4.20 m (frozen) | 4.40 m | −0.20 m (drifted back) |

When drift approached `L1 + L2 = 0.4394 m` (kinematic limit), IK clamped, PD saturated → flip.

**Fix: full rewrite to CHAMP body-frame algorithm.**  
Foot targets are now computed **fresh from `_ZERO_STANCE` every tick** — no world-frame state is stored.  
- Stance: backward sweep `sx = (L/2) × (1 − 2×tp)` in body frame.  
- Swing: 12-point Bezier arc scaled by `step_length`.  
- IK is always evaluated at a body-frame position near nominal → no singularity possible.

This is the approach in the C++ CHAMP source (`trajectory_planner.h`) and the reason CHAMP is stable across arbitrary body displacements.

---

### 4 — Bug: wheels slipping (attempt 1 — freewheel touchdown brake)

**Symptom:** Visible wheel slip during trot, especially on leg touchdown.

**Initial diagnosis:** MuJoCo velocity actuator model: `τ = kv × (ctrl − ω_actual)`. With `kv=5` and freewheel (`ctrl = ω_prev`), a swinging wheel (ω ≈ 0 from cold start) lands and is instantly forced to `ω = vx/r`. The transient brake torque was estimated at ~11.5 Nm per wheel.

**Attempted fix:** Replace freewheel with `_cmd_to_wheel(vx_cmd, wz)` — command all wheels at the nominal rolling speed for the nav setpoint (`vx=0.2 m/s → ctrl = 2.33 rad/s`).

---

### 5 — Regression: robot moves backward catastrophically

**Symptom:** Robot moved −0.757 m in 3 s when commanded forward at `vx=0.2 m/s`.

**Root cause:** A variation that used *actual body velocity* (`linvel projected onto body X`) for wheel command. Any transient backward body motion (gait lurch) made the wheels actively drive backward → positive feedback runaway.

**Fix:** Reverted to *commanded* velocity: `_cmd_to_wheel(self._vx, self._wz)`.

---

### 6 — Bug: wheels still slipping (commanded speed wrong order of magnitude)

**Symptom:** Wheel slip still clearly visible.

**Root cause (corrected analysis):** Actual trot body speed ≈ 0.024 m/s → `ω_roll = 0.28 rad/s`. But `_cmd_to_wheel(0.2)` commanded `ctrl = 2.33 rad/s`. Drive torque = `kv × (2.33 − 0.28) = 10.25 Nm` per wheel — the wheels were actively grinding at **88% slip**. The original 11.5 Nm calculation was wrong because it used commanded velocity instead of actual trot body speed.

**Corrected freewheel analysis:**  
- In stance: wheel rolls at `ω = v_body/r ≈ 0.28 rad/s`.  
- In swing (air): `ctrl = ω_prev ≈ 0.28 rad/s` is carried forward — wheel keeps spinning.  
- On touchdown: `ctrl ≈ ω_actual` → `τ_brake ≈ 0`.

**Fix:** Reverted to freewheel — `self._env.set_wheel_velocities(self._env.get_wheel_velocities())`.

```python
# In _trot_step():
# Freewheel: ctrl = ω_prev.  In stance ω≈0.28 rad/s; swing carries that
# value forward → touchdown τ_brake ≈ 0.  Commanding vx_cmd=0.2 m/s gives
# ctrl=2.33 rad/s vs actual ω=0.28 rad/s → 10 Nm grinding → 88% slip.
self._env.set_wheel_velocities(self._env.get_wheel_velocities())
```

---

### 7 — Bug: occupancy grid mapping false obstacles (floor hits)

**Symptom:** A* planner frequently logged `[CHAMP] No path to (x, y)` even for reachable locations. CFPA2 oscillated between `executing` and `no_reachable` every few seconds.

**Root cause:** `OccupancyMapper.update()` had `rz = 0.0` hardcoded. The LiDAR scans at −7° depression from height ~0.40 m above the floor; floor hits have `world_z ≈ 0–0.09 m`. With `rz=0`, `height_above_robot = world_z − 0 ≈ +0.09 m`, which **passes** the `min_scan_height = −0.05 m` filter → floor mapped as occupied cells.

**Measured impact:** At spawn (body_z = 0.60 m), **15 of 34 scan points (44%)** were floor hits. With `rz=0`: 29 occupied cells per scan. With corrected `rz=0.60`: 10 occupied cells per scan. **19 false obstacles per scan removed** (66% reduction).

```
Floor hits removed: 22 / 34 scan points
False occupied cells: 29 → 10 per scan
```

**Fix:** Added `robot_z` parameter to `OccupancyMapper.update()`:

```python
# occupancy_grid.py
def update(self, points_world, robot_pose, robot_z: float = 0.0) -> None:
    rx, ry, _ = robot_pose
    rz = robot_z  # was hardcoded 0.0 — floor hits at world_z≈0 were mapped as obstacles
```

Both `main_champ.py` and `main.py` now pass `env.get_pose3d()[2]`:

```python
body_z = env.get_pose3d()[2]
mapper.update(pts, pose, robot_z=body_z)
```

---

### 8 — Feature: live occupancy map window

**Goal:** Visualise the occupancy grid, robot pose, A* path, and current CFPA2 goal in real time for debugging.

**Attempt 1 — `matplotlib` TkAgg:** Crashed immediately:
```
NSInvalidArgumentException: -[NSApplication macOSVersion]: unrecognized selector
```
Root cause: `mjpython` runs Python in a background `pthread`. macOS forbids all GUI operations (AppKit, Tk, Qt) on non-main threads.

**Solution — `cv2` render to PNG + macOS Preview auto-refresh:**  
`cv2.imwrite()` is just a file write — no GUI, no thread restrictions. Preview opens the file once and auto-refreshes whenever the PNG changes (macOS native behaviour).

```python
class MapViz:
    def __init__(self, mapper):
        tmp = tempfile.mktemp(suffix=".png", prefix="occ_map_")
        self._png = Path(tmp)
        cv2.imwrite(str(self._png), blank)
        subprocess.Popen(["open", str(self._png)])   # opens Preview

    def update(self, robot_pose):
        # build numpy image from log-odds grid
        # draw robot (red dot + heading), path (green), goal (yellow X)
        cv2.imwrite(str(self._png), img)  # Preview auto-refreshes
```

Map is updated every LiDAR scan (rate controlled by `lidar.publish_rate_hz` in `config.yaml`, default 5 Hz).

Disable with `--no-map` flag. Enabled by default when not `--headless`.

Colour key:

| Colour | Meaning |
|---|---|
| Mid-gray | Unknown (unobserved) |
| White | Free space |
| Near-black | Occupied / wall |
| Red dot + tick | Robot position + heading |
| Green line | Active A* path |
| Yellow × | Current CFPA2 navigation goal |

---

## Current open issues

### A — "Exploration complete — stopping." spam (not truly stopping)

**Observed:** After the stop trigger fires (`metrics.tick()` returns `True`), the log line `[CHAMP] Exploration complete — stopping.` is printed **every 0.5 s** indefinitely. The robot does halt (`set_cmd_vel(0,0,0)`) but the loop never exits.

**Log excerpt:**
```
17:59:22 INFO  main_champ | [CHAMP] Exploration complete — stopping.
17:59:23 INFO  main_champ | [CHAMP] Exploration complete — stopping.
17:59:24 INFO  main_champ | [CHAMP] Exploration complete — stopping.
  ... (repeating every ~0.5 s for >60 s)
```

**Root cause:** In `run_loop`, the stop trigger returns `True` only on the first invocation (it sets an internal `_done = True`). But after that, `metrics.tick()` keeps calling `_done` internally and returning `True` on every subsequent call too (or the stop condition keeps re-evaluating True). The loop never `break`s.

**Needed fix:** After the stop trigger fires, `run_loop` should `break` out of the `while True` loop (or close the viewer). Current code only calls `set_cmd_vel(0,0,0,False)` but continues looping.

```python
# Current (broken):
done = metrics.tick()
if done:
    log.info("[CHAMP] Exploration complete — stopping.")
    leg_ctrl.set_cmd_vel(0.0, 0.0, 0.0, False)

# Needed:
done = metrics.tick()
if done:
    log.info("[CHAMP] Exploration complete — stopping.")
    leg_ctrl.set_cmd_vel(0.0, 0.0, 0.0, False)
    break  # ← exit the simulation loop
```

---

### B — CFPA2 CPU warning (~160% single-core)

**Observed:**
```
WARNING cfpa2_single_robot | PERF threshold exceeded: CPU 160.0% > 15.0% single-core budget
```

CFPA2's adaptive stride (currently `stride=2`) reduces computation but CPU is still high. On an M-series Mac this doesn't cause real slowdown, but it's worth watching for real-robot deployment (Jetson) where single-core budget matters.

**Not blocking.** Known issue from the ROS 2 stack too.

---

### C — Trot body speed (~0.024 m/s) much lower than commanded (0.2 m/s)

**Observed:** In 3 s with `vx=0.2 m/s` commanded, the robot moved `Δx ≈ 0.072 m` → actual `≈ 0.024 m/s`.

**Likely cause:** Raibert `step_length = (T_stance / 2) × vx = 0.125 × 0.2 = 0.025 m`. Each step only covers 2.5 cm. At 2 steps/s (trot at `gait_period=0.5 s`), max speed from gait mechanics alone is `0.025 × 4 = 0.05 m/s` before slip and slippage deductions. Wheels freewheel (no active drive) so all body speed comes from leg push-off.

**Options:**
1. Increase `gait_period` → longer steps → higher speed.
2. Enable wheel drive in trot (hybrid: legs for body support, wheels for propulsion) — requires careful integration to avoid the earlier slip regressions.
3. Accept the low trot speed and use it for the pure-locomotion demo; map exploration still works at 0.024 m/s.

---

### D — No `min_scan_height` in `config.yaml`

`OccupancyMapper` has a `min_scan_height` init param (default `−0.05 m`) but `config.yaml` only exposes `max_scan_height`. After the `robot_z` fix, `min_scan_height` is applied correctly at the default value, but it can't be tuned at runtime without editing the source.

**Needed fix:** Add `min_scan_height` to the `mapping:` block in `config.yaml` and thread it through `OccupancyMapper()` init in both entry points.

---

## Quick reference: run commands

```bash
# CHAMP trot + map window
mjpython main_champ.py

# headless (no MuJoCo viewer, no map window)
mjpython main_champ.py --headless

# CHAMP trot + no map window
mjpython main_champ.py --no-map

# Baseline (wheels only, LegController)
mjpython main.py
```

Config lives in `standalone/config.yaml` — YAML + Python changes take effect immediately (no rebuild).

---

## File map

```
standalone/
├── main.py                   Baseline (LegController + wheels)
├── main_champ.py             CHAMP trot entry point
├── config.yaml               All tunable params
├── sim/
│   ├── mujoco_env.py         MuJoCo wrapper (pose, joints, LiDAR trigger)
│   ├── champ_controller.py   CHAMP port (Raibert + Bezier + IK + PD)
│   ├── leg_controller.py     Baseline PD+wheel controller
│   └── sensor/lidar.py       Raycasting LiDAR
├── mapping/
│   └── occupancy_grid.py     Log-odds 2D mapper (Bresenham ray-trace)
├── nav/
│   ├── planner.py            A* with obstacle inflation
│   └── controller.py         Pure-pursuit path follower
├── exploration/
│   └── cfpa2_single_robot.py CFPA2 frontier allocator (standalone port)
├── control/
│   ├── hybrid_cmd_router.py  leg↔wheel mode switcher
│   ├── cfpa2_bridge.py       CFPA2 goal → A* bridge
│   └── stuck_watchdog.py     backup-recovery trigger
└── observability/
    └── metrics_logger.py     Stop trigger + progress log
```
