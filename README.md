# Collab_QRC — Multi-Robot Autonomous Exploration

Multi-robot autonomy on Unitree **Go2 / Go2W** wheeled-legged quadrupeds. ROS 2 Humble + **MuJoCo** (primary sim) + **real-robot** deployment (CycloneDDS, Mid-360 LiDAR). Gazebo Classic path retained as legacy.

Two active threads:

- **Single-robot nav + exploration benchmarking** — Fast-LIO2 + SC-PGO SLAM, CMU autonomy stack (terrain_analysis + localPlanner + pathFollower) with either **CFPA2** or the **real CMU TARE planner** for goal sourcing.
- **Dual-robot door task** — two Go2Ws in adjacent rooms divided by a spring-loaded fire door; VLM-driven coordination with an analytical door-lock barrier.

Detailed index and per-topic docs: **[CLAUDE.md](CLAUDE.md)** + [docs/claude/](docs/claude/).

## Quick Start

```bash
git clone <repo-url> Collab_QRC && cd Collab_QRC
./setup.sh                           # ROS 2 Humble + conda env + deps + colcon build

micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Sim launches

```bash
# Dual-robot door task (VLM coordination, MuJoCo)
./scripts/launch/door_demo_mujoco.sh

# Single-robot VLM exploration (Phase 1)
./scripts/launch/vlm_demo_mujoco.sh

# Nav stack smoke + benchmarks (Go2W, reactive/FAR backends)
./scripts/launch/nav_test_mujoco.sh
./scripts/bench/benchmark_far_nav.sh          # 5 trials, PASS = cov≥90% ∧ contacts==0
./scripts/bench/benchmark_fastlio.sh

# Go2 (non-W) with CHAMP — demo1 (12×8) / demo3 (24×16)
./scripts/launch/nav_test_go2.sh            gui:=true rviz:=true
./scripts/launch/nav_test_go2_demo3.sh      gui:=true rviz:=true

# Go2 + real CMU TARE exploration (FAR bypassed, watchdog armed)
./scripts/launch/nav_test_go2_tare_real.sh  gui:=true rviz:=true
./scripts/bench/benchmark_go2_tare.sh       # 10 trials × 10 min on demo3
```

### Real-robot launches

```bash
# Go2W — Ethernet, Cartographer + L1, CFPA2 (defaults)
./scripts/real/real_autonomy.sh

# Go2W — Livox Mid-360 + Fast-LIO2, FAR nav
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far

# Go2 (non-W, walking gait) — same stack
./scripts/real/real_autonomy_go2.sh slam=fastlio_mid360 nav=far

# **Real CMU TARE** exploration → localPlanner direct (FAR unwired).
# oa=false is REQUIRED: default oa=true routes Move to /api/obstacles_avoid/request
# (needs manual-mode pre-arm); oa=false routes to /api/sport/request (api_id=1008).
./scripts/real/real_autonomy.sh robot=go2 slam=fastlio_mid360 nav=tare_real oa=false

./scripts/real/real_autonomy.sh stop          # tear down everything
```

Debug dashboards auto-start:

- Door task: <http://127.0.0.1:8080>
- VLM exploration: <http://localhost:8501>

## Repository Structure

```text
Collab_QRC/
  src/
    go2w/                              Go2W/Go2 platform packages
      go2_gazebo_sim/                    MJCF scenes, TARE configs, sim launch files
      mujoco_sensor_bridge/              MuJoCo sensor nodes (LiDAR, contact, odom)
      go2w_nav/ go2w_control/            Reactive RRT*, A*, cmd routing
      go2w_perception/ go2w_config/      QoS bridge, Cartographer bridge, shared YAML
      go2w_safety/                       Safety monitors (wall checker, supervisor)
      go2w_real_bringup/                 Real-robot launches (real_single*, tare_real)
      unitree_go2w_ros2/                 Unitree ROS 2 SDK integration
    exploration/
      cfpa2_collaborative_autonomy/      CFPA2 frontier allocator (single-robot)
      go2_nav_algorithms/                Scan mapper, frontier detection, pipeline
    collaborative_exploration/
      door_task/                         Dual-robot door task (VLM + analytical barrier)
    vlm_explorer/                        VLM-in-the-loop exploration (Phase 1)
    vendor/                              Third-party sources (vendored, not submodules)
      tare_planner/                        CMU TARE (caochao39, humble-jazzy)
      autonomy_stack_go2/                  CMU FAR + terrain_analysis + localPlanner
      fast_lio/                            Fast-LIO2 LiDAR-inertial SLAM
      mujoco_ros2_control/                 DFKI MuJoCo ros2_control HW interface
      champ/                               CHAMP quadruped locomotion controller
      livox_ros_driver2/ Livox-SDK2/       Mid-360 driver (workspace-local install)
  scripts/
    launch/      user-invoked entry points (nav_test_*, door_demo, vlm_demo)
    bench/       multi-trial PASS-criterion runners + session_reporter
    runtime/     ROS 2 nodes started by launches (watchdog, spawners, bridges)
    debug/       live-stack observation tools
    ops/         one-shot dev/ops utilities
    real/        real-robot entry points
  config/        DDS configs (fastdds_no_shm.xml)
  docs/claude/   Per-topic engineering notes (door_task, nav_benchmarks, go2_integration, real_robot, …)
  setup.sh
```

## Architecture (sim path)

```text
MuJoCo physics (500 Hz)
  └─ mujoco_ros2_control  →  LiDAR PointCloud2, joint_states, /clock
         │
         ├─ Fast-LIO2  →  /Odometry, /cloud_registered
         │     └─ SC-PGO loop closure  →  /aft_pgo_odom
         │
         ├─ terrain_analysis  →  /terrain_map[_ext]
         │
         ├─ exploration goal source
         │     ├─ CFPA2 frontier allocator          (nav_backend=reactive/far/default)
         │     └─ real CMU TARE planner             (nav_backend=tare_real, FAR unwired)
         │           └─ waypoint watchdog → /nogo_boundary (persistent blacklist)
         │
         └─ localPlanner  →  pathFollower  →  /cmd_vel
                                                 └─ CHAMP (200 Hz IK) → ros2_control
```

## Navigation Backends

Switchable via `nav_backend:=` / `nav_execution_backend:=` at launch time.

| Backend | Planner | Note |
| --- | --- | --- |
| `astar` | `astar_nav_node` | C++ A\* + pure-pursuit + Stanley + oriented footprint validation (Option B) |
| `default` | `default_nav.py` | Python A\* grid + D\* Lite + recovery (legacy stable) |
| `far` | CMU autonomy stack | Terrain analysis + FAR V-graph + pathFollower |
| `tare_real` | Real CMU TARE → localPlanner direct (Go2 only, FAR unwired) | Used in `nav_test_go2_tare_real.launch.py` and `real_single_tare_real.launch.py` |

Legacy aliases silently upgrade: `reactive` → `default`, `rrt_star` / `far_rrt_star` / `mppi` → `astar`. The reactive RRT\* planner (`reactive_nav_node`) and MPPI (`mppi_nav_node`) were deleted 2026-04-24.

**Why TARE bypasses FAR:** FAR's V-graph is built over *observed traversable* space; TARE's frontier goals sit at the *boundary* of observed space. Stacking them runs two global planners with conflicting scopes. CMU's reference pipeline pairs TARE → localPlanner directly.

## SLAM Options

| Backend | Topic in | Backend chain | Use |
| --- | --- | --- | --- |
| Cartographer 2D | `scan_3d` (LaserScan) | `cartographer_node` → TF → `carto_odom_bridge` | Default on Go2W real robot (L1 LiDAR) |
| Fast-LIO2 + SC-PGO | PointCloud2 | `fast_lio` → `sc_pgo` → `slam_odom_relay` | Mid-360 LiDAR, sim + real; fixes scan-odom temporal lag |

## VLM Integration

Cloud VLM queried with rendered occupancy maps + camera images; decides exploration goals or emits FSM coordination plans.

| Provider | Env var | Default model |
| --- | --- | --- |
| xAI | `XAI_API_KEY` | grok-4-1-fast-non-reasoning |
| OpenAI | `OPENAI_API_KEY` | gpt-4o-mini |
| Anthropic | `ANTHROPIC_API_KEY` | — |

Keys go in `.env.xai` at repo root (or exported).

## Build

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Vendored COLCON_IGNORE markers are committed under paths listed below (.gitignore
# whitelists those files). No manual `touch` needed unless you add a new skip marker.

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

| Path | Why ignored |
| --- | --- |
| `src/vendor/autonomy_stack_go2/` | CMU upstream, vendored as reference; the active FAR / terrain_analysis / localPlanner builds live in their own packages. |
| `src/vendor/Livox-SDK2/` | Plain CMake library, not a colcon package; colcon would expose it as `livox_sdk2` and clash with the workspace-local install used by `livox_ros_driver2`. |
| `src/vendor/sc_pgo/fast_lio_sam/` | ROS 1 (catkin) — see `PORT_TO_ROS2.md` next to it. |
| `src/collaborative_exploration/go2_tare_planner_ros2/generated/tare_planner/` | ROS 1 catkin output sitting next to ROS 2 `go2_tare_planner_ros2`; would duplicate the package name `tare_planner` vs `vendor/tare_planner`. |

YAML + Python are live via symlink-install; C++ requires rebuild.

Before `colcon build`, optional sanity check (duplicate package names + CMake↔package.xml deps):

```bash
python3 scripts/ops/workspace_colcon_audit.py
```

If CMake reports `Could not find package X` for a **workspace** package, build dependencies first:
`colcon build --symlink-install --packages-up-to <that_pkg>`.

## Golden Rules

1. Always `use_sim_time: true` for every node in sim. Mixed time domains corrupt maps.
2. Never use stale TF fallback (`tf2::TimePointZero`). Drop the scan instead.
3. Each scan painted exactly once — clear `last_scan_` after processing.
4. Dual-robot TF must be namespaced: remap `/tf` → `/{ns}/tf` for every node.
5. Kill zombie MuJoCo before re-launch (see [debug_notes.md](docs/claude/debug_notes.md)).
6. Benchmark PASS = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. Use ≥10 trials for reliability claims.
7. Real-robot: any Unitree BT pad button press latches a 5 s **supervisor-panic** window — auto `cmd_vel` blocked, FAR disarmed, sticks drive directly. See [real_robot.md](docs/claude/real_robot.md#supervisor-panic-override-any-button-emergency).

## Debugging

```bash
ros2 topic hz /{ns}/registered_scan
ros2 topic hz /{ns}/odom/nav

# Stale DDS state → clear shared memory
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

# Zombie MuJoCo cleanup
pkill -9 -f mujoco_ros2_control; pkill -9 -f mujoco_sensor; killall -9 rviz2
```

See [docs/claude/debug_notes.md](docs/claude/debug_notes.md) for cross-cutting gotchas (QoS, zombies, MuJoCo quirks) and [docs/claude/real_robot.md](docs/claude/real_robot.md) for the 10-layer Mid-360 bring-up chain.

## Environment

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble
- **Python:** 3.10 (micromamba `cmu_env`)
- **Sim:** MuJoCo 3.6.0 (pip) + DFKI `mujoco_ros2_control`
- **DDS:** FastDDS (sim, `config/fastdds_no_shm.xml`) · CycloneDDS (real robot)
- **Build:** colcon + ament_cmake / ament_python

## License

Research project. Contact maintainers for licensing.
