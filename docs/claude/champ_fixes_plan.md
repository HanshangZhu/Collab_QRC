
# Standalone CHAMP — Fix Plan (2026-05-14)

Three workstreams. Run in order; each is independent enough to verify before moving on.

Reference docs: [`champ_standalone.md`](champ_standalone.md), [`champ_standalone_dev_log.md`](champ_standalone_dev_log.md).

---

## A. Wheel slip fix — architecture mismatch

### Root cause

[`standalone/main_champ.py:397-401`](../../standalone/main_champ.py#L397-L401) discards router's `wheel_mode` flag and hardcodes `False`. [`standalone/config.yaml:119`](../../standalone/config.yaml#L119) sets `wheel_mode_speed: 1.0` so `ChampController.step()` never auto-switches to `_wheel_drive_step` (nav cap is 0.5 m/s).

Result: robot always trots. Trot freewheels wheels via `set_wheel_velocities(get_wheel_velocities())`. Wheel collision is `condim=6 friction="0.8 0.02 0.01"` (high rolling resistance). Leg stance push converts to body slide instead of wheel roll → 88% slip (open issue C in dev log: 0.024 m/s actual vs 0.2 m/s commanded).

### Original ROS2 design

[`archive/ros2_legacy/go2w_config/config/control/go2w_hybrid_motion.yaml`](../../archive/ros2_legacy/go2w_config/config/control/go2w_hybrid_motion.yaml):

```
wheel_linear_threshold: 0.18
wheel_engage_sustain_sec: 0.5
wheel_mode_hold_sec: 0.5
legged_mode_hold_sec: 0.8
legged_override_curvature: 1.0
```

At cruise speed (≥ 0.18 m/s sustained 0.5 s) router switches to **wheel mode** → CHAMP receives `cmd_vel_legged=0`, legs hold standing pose, wheels actively driven via separate `wheel_velocity_controller`. Trot used only for low-speed pivots and idle.

[`archive/ros2_legacy/go2w_control/scripts/go2w_hybrid_cmd_router.py:281-292`](../../archive/ros2_legacy/go2w_control/scripts/go2w_hybrid_cmd_router.py#L281-L292) — `freewheel_in_legged=True` default; in legged/idle mode publishes mirror-ω to neutralise actuator brake torque.

### Fix

1. [`main_champ.py`](../../standalone/main_champ.py): honor router output.
   ```python
   vx, wz = controller.compute(pose)
   router.set_cmd_vel(vx, 0.0, wz)
   vx, wz, wheel_mode = router.tick()
   vy = 0.0
   leg_ctrl.set_cmd_vel(vx, vy, wz, wheel_mode)
   ```

2. [`config.yaml`](../../standalone/config.yaml): `wheel_mode_speed: 1.0 → 0.18` (or 0.0 to fully defer to router).

`ChampController._wheel_drive_step()` already implements correct behavior (legs PD-hold `q_stand`, wheels `_cmd_to_wheel(vx, wz)`). No controller-side change needed.

### Acceptance

- Robot at vx=0.5 m/s cmd reaches ≈ 0.45 m/s actual (within 10%).
- Router status logs show `wheel` mode dominant during cruise.
- Slip visible only at startup / direction changes.

---

## B. CHAMP gait fidelity — copy original params + missing functions

### Param sync ([go2w/gait.yaml](../../src/go2w/go2_gazebo_sim/config/champ/go2w/gait.yaml) + [champ_config gait.yaml](../../archive/ros2_legacy/champ/champ_config/config/gait/gait.yaml))

Edit [`standalone/config.yaml`](../../standalone/config.yaml) `champ:` block:

| Param | Current | Original Go2W | Action |
|---|---|---|---|
| `swing_height` | 0.08 | 0.04 | → 0.04 |
| `stance_depth` | 0.00 | 0.01 | → 0.01 |
| `gait_period` | 0.50 | stance 0.25 + swing 0.25 (FIXED) | keep 0.50 |
| `duty_factor` | 0.50 | derived 0.5 | keep |
| `body_height` | 0.36 | `nominal_height: 0.28` | → 0.28 |

### Joint PD ([go2w_mujoco_controllers.yaml:34-45](../../src/go2w/go2_gazebo_sim/mujoco/go2w_mujoco_controllers.yaml#L34-L45))

Original uses uniform `p=100, i=0.2, d=1.0, ff_velocity_scale=1.0` @ 200 Hz `joint_trajectory_controller`.

Standalone has `kp_leg=60, kd_leg=3` @ 500 Hz, no integral, no FF.

Action: try `kp_leg: 100, kd_leg: 1.0`. Skip integral initially (trot dynamics rarely accumulate steady-state error worth fixing). Add optional `ki_leg + i_clamp` only if drift visible.

### Missing functions to port

Implement faithfully from C++ source:

1. **`PhaseGenerator.run()`** — time-based, restarts on motion onset.
   - Source: [`phase_generator.h:60-129`](../../archive/ros2_legacy/champ/champ/include/champ/leg_controller/phase_generator.h#L60-L129).
   - **Critical**: `swing_phase_period = 0.25 s HARDCODED` (line 63), independent of `stance_duration`.
   - Resets `last_touchdown_` on `target_velocity == 0` so gait restarts cleanly after stops.
   - Replace `_global_phase` field + advance in [`champ_controller.py:_trot_step`](../../standalone/sim/champ_controller.py).

2. **First-stride suppression** ([`phase_generator.h:118-128`](../../archive/ros2_legacy/champ/champ/include/champ/leg_controller/phase_generator.h#L118-L128)).
   - `has_swung_` flag: until first stance crosses 0.5, zero out FL+RR stance and FR+RL swing.
   - Prevents cold-start lurch.

3. **`BodyController.poseCommand()`** ([`body_controller.h:48-92`](../../archive/ros2_legacy/champ/champ/include/champ/body_controller/body_controller.h#L48-L92)).
   - Adjusts foot Z target by `nominal_height` at runtime; clamps `req_translation_z ∈ [0, −zero_stance.Z × 0.65]`.
   - Standalone currently bakes body height into `_ZERO_STANCE` constant. Port so `body_height` is live param like CHAMP.

4. **`TrajectoryPlanner.generate` prev_foot fallback** ([`trajectory_planner.h:154-159`](../../archive/ros2_legacy/champ/champ/include/champ/leg_controller/trajectory_planner.h#L154-L159)).
   - When both `swing_phase_signal == 0 && stance_phase_signal == 0 && step_length > 0`, reuse `prev_foot_position_`.
   - Minor correctness fix at gait-transition boundaries.

5. **Target velocity computation** ([`leg_controller.h:100-101`](../../archive/ros2_legacy/champ/champ/include/champ/leg_controller/leg_controller.h#L100-L101)).
   - `tangential_velocity = wz × center_to_nominal`
   - `velocity = sqrt(vx² + (vy + tangential)²)` — pass to `phase_generator.run()` as target.
   - Replace standalone's boolean `is_moving`.

6. **Loop rate parity** — original CHAMP runs at 200 Hz ([`quadruped_controller.cpp:49`](../../archive/ros2_legacy/champ/champ_base/src/quadruped_controller.cpp#L49)). Standalone runs gait calc every 500 Hz physics tick. Phase advances correctly via `dt`, but PD acts 2.5× more frequently. Consider rate-limiting gait calc to 200 Hz with leg PD still @ 500 Hz, OR re-tune PD for 500 Hz.

### Acceptance

- Body height stable at 0.28 m ± 1 cm during trot (matches `nominal_height`).
- No first-stride lurch.
- Stop/start cycles don't accumulate phase drift.
- Trot stable at 0.1 m/s for ≥ 60 s without tip.

---

## C. Live occupancy map — kill PNG+Preview, use proper window

### Why current approach fails

[`main_champ.py:96-200`](../../standalone/main_champ.py#L96-L200) `MapViz` uses `cv2.imwrite(/tmp/occ_map_*.png)` + `subprocess.Popen(["open", png])` → macOS Preview auto-refresh. Works but isn't a real live window. Tk/Qt crash under `mjpython` because Python runs in a background pthread (macOS forbids GUI on non-main thread).

### Approach: subprocess + memmap

Child Python process owns its main thread → GUI libs work fine.

```
main_champ.py (mjpython)
  └─ MapWriter
       writes numpy.memmap → /tmp/standalone_map.bin
  └─ subprocess.Popen([sys.executable, "-m", "standalone.viz.map_viewer", path])
       (regular python3, NOT mjpython)

map_viewer.py
  reads memmap @ 10 Hz
  matplotlib FuncAnimation window
```

### Memmap layout (single file, fixed offsets)

```
Header (256 B):
  magic      uint32   0xCAFE5A11
  width      int32
  height     int32
  resolution float32
  origin_x   float32
  origin_y   float32
  robot_x    float32
  robot_y    float32
  robot_yaw  float32
  goal_x     float32
  goal_y     float32
  has_goal   uint8
  path_len   int32        (0..MAX_PATH)
  status[32] char         (e.g. "searching", "executing")
  frame_id   uint64       (incremented each write)

Grid:  H × W   int8       (−1 unknown, 0 free, 100 occ)
Path:  256 × 2  float32   (x, y world)
```

`frame_id` lets child poll just the header to detect changes.

### New files

- [`standalone/observability/map_writer.py`](../../standalone/observability/map_writer.py) (new)
  - `MapWriter(mapper, path="/tmp/standalone_map.bin", max_path=256)`
  - Methods: `tick(robot_pose)`, `set_path(path)`, `set_goal(x,y)`, `clear_goal()`, `set_status(s)`, `close()`
  - Writes incrementally; no global recopy each tick.

- [`standalone/viz/__init__.py`](../../standalone/viz/__init__.py) (new, empty)
- [`standalone/viz/map_viewer.py`](../../standalone/viz/map_viewer.py) (new)
  - Argv: `memmap_path`
  - Loads memmap RO, `matplotlib FuncAnimation` 10 Hz.
  - Overlays: grid (cmap), robot dot+heading, path polyline, goal X, status text.
  - Closes cleanly on window close.

### main_champ.py changes

- Replace `MapViz` class with `MapWriter` (~10 lines).
- Spawn subprocess at startup; `atexit.register` to terminate child.
- `--no-map` flag preserved.

### Acceptance

- Window opens on launch, refreshes ≥ 5 Hz.
- Robot, path, goal track scene state.
- Closing window doesn't kill parent.
- Parent exit kills child.

---

## Execution order

1. **Wheel slip fix (A)** — smallest change, biggest immediate win.
2. **Verify** A — observe wheel-mode engagement at cruise; measure slip.
3. **CHAMP params (B param sync only)** — config-only edits, then re-test trot.
4. **Live map (C)** — independent, ship next.
5. **Periodic replan + path invalidation (D)** — A* re-runs at 1 Hz; path validated each lidar tick; blocked + no-replan → controller stops + watchdog recovers.
6. **CHAMP missing functions (B port work)** — last; only if trot still wonky after params.

Each step independently verifiable. Roll back any single step without affecting others.

---

## D. Periodic replan + path invalidation

### Symptom

After reaching goal 1, robot drove through a wall trying to reach goal 2. Path was planned once on goal-callback against the map *at that instant*. Wall ahead was still **unknown** (`UNKNOWN_PASSABLE=True` in [`planner.py:30`](../../standalone/nav/planner.py#L30)) so A* picked a straight line through it. Once the wall was scanned, no replan → robot drove the stale path into it.

### Original ROS2 behavior

[`default_nav.py:70-77`](../../src/go2w/go2w_nav/scripts/default_nav.py#L70-L77) — `AsyncGridPlanner(inflation_m=0.38, replan_interval_sec=2.0)`; [`default_nav.py:_try_grid_plan`](../../src/go2w/go2w_nav/scripts/default_nav.py#L268) runs every control-loop tick. Cached goal, fresh A* against fresh map every cycle.

### Fix (3 edits)

1. [`standalone/config.yaml`](../../standalone/config.yaml): `planner_inflation_cells: 3 → 4` (0.30 m → 0.40 m, matches original 0.38 m).

2. [`standalone/main_champ.py`](../../standalone/main_champ.py) + [`standalone/main.py`](../../standalone/main.py): cache active goal in shared state, add `_maybe_replan()` called from lidar tick. Replans when:
   - Periodic timer fires (`REPLAN_PERIOD_SEC = 1.0`), OR
   - Sampled path waypoints intersect any cell with value ≥ 50 (occupied).

3. When path is blocked and replan returns no path: `controller.clear()` so cmd_vel goes to zero and `StuckWatchdog` can trigger backup recovery.

### Acceptance

- After reaching goal 1, robot starts toward goal 2; if a wall enters scan range, log line `[blocked] path to (gx, gy) — N wps` appears within 1 s; path overlay in viewer redraws.
- Robot never drives through a known-occupied cell.
- If replan fails and watchdog cooldown elapses, robot reverses and CFPA2 may pick alternate frontier.
