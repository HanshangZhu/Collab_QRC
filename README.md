# Collab_QRC — Multi-Robot Autonomous Exploration

Multi-robot autonomy on Unitree **Go2 / Go2W** wheeled-legged quadrupeds. ROS 2 Humble + **MuJoCo** (primary sim) + **real-robot** deployment (CycloneDDS, Mid-360 LiDAR). Gazebo Classic path retained as legacy.

Two active threads:

- **Single-robot nav + exploration** — Fast-LIO2 / Cartographer SLAM, **Nav2 MPPI / SmacPlannerLattice (SE2 holonomic)** as production stack since 2026-04-29; CMU autonomy stack (FAR / TARE) and Python A* kept as escape hatches.
- **Dual-robot door task** — two Go2Ws in adjacent rooms divided by a spring-loaded fire door; VLM-driven coordination with an analytical door-lock barrier.

A third **ROS 1 hybrid SLAM** thread (Swarm-LIO2 + Dynamic-LIO + ERASOR via Docker + ros1_bridge) is wired in additively (Phase A) — see `docker/ros1_hybrid_slam/`.

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
# Single-robot nav + exploration (Nav2 MPPI + Fast-LIO2 + CFPA2 — production)
./scripts/launch/nav_test_demo3.sh                        # demo3 24×16 m, 4-quadrant scene
./scripts/launch/nav_test_fastlio.sh                      # demo1 12×8 m
./scripts/launch/nav_test_demo2.sh                        # LRC maze

# Real CMU TARE exploration (FAR bypassed, TARE → localPlanner direct)
./scripts/launch/nav_test_go2_tare_real.sh  gui:=true rviz:=true

# ROS1 hybrid Swarm-LIO2 in docker, fed by MuJoCo lidar via ros1_bridge
./scripts/launch/nav_test_swarm_lio2_se2.sh

# Dual-robot door task (VLM coordination, MuJoCo)
./scripts/launch/door_demo_mujoco.sh

# Single-robot VLM exploration (Phase 1)
./scripts/launch/vlm_demo_mujoco.sh

# Multi-trial benchmarks (PASS = cov≥90% ∧ contacts==0 ∧ ¬tipped, ≥10 trials for stats)
./scripts/bench/benchmark_fastlio.sh                       # FAR + Fast-LIO2 + Mid-360
./scripts/bench/benchmark_far_nav.sh                       # FAR + Cartographer + L1
./scripts/bench/benchmark_go2_tare.sh                      # 10 trials × 10 min, Go2 + TARE
```

### Real-robot launches

```bash
# Recommended entry — SE2 holonomic baked in, sport API direct (oa=false), curated flags.
./scripts/real/real_autonomy_se2.sh                                # default Go2W, Cartographer + L1
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360             # Mid-360 + Fast-LIO2
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360 lidar_range=4.0
./scripts/real/real_autonomy_se2.sh stop                            # tear down everything

# Full-surface launcher (legacy/comparison runs):
./scripts/real/real_autonomy.sh                                     # nav2_mppi default
./scripts/real/real_autonomy.sh nav=far slam=fastlio_mid360          # FAR escape hatch
./scripts/real/real_autonomy.sh holonomic_profile=se2_holonomic     # Go2W or Go2 walking SE2

# Go2 (non-W, walking gait):
./scripts/real/real_autonomy_go2.sh                                 # nav2_mppi default
./scripts/real/real_autonomy_go2.sh nav=tare_real oa=false          # CMU TARE → localPlanner

# Outdoor SLAM-debug bag tools (no autonomy, BT pad walks the robot):
./scripts/real/record_livox_dataset.sh tag=outdoor_run1              # /livox/lidar+imu+FLIO output → MCAP/sqlite3
./scripts/real/replay_livox_dataset.sh                               # autonomy.rviz parity, /tf_static QoS fixed
./scripts/debug/analyze_lidar_odom_drops.py                          # per-scan latency, dropped-frame stats
```

### Onboard autonomy — full stack native on the Go2 Orin NX (ROS 1 Noetic)

The **entire** autonomy stack runs onboard the real Go2's Jetson Orin NX 16 GB in
native ROS 1 Noetic — **no ros1_bridge in the data path**. SLAM (Point-LIO) +
traversability (elevation_mapping_cupy + CNN) + nav (Nav2 SmacLattice + **CUDA-MPPI**
ported to `move_base`) + CFPA2 frontier exploration (C++). Verified real-time on
the bench (CPU < 33 %, GPU < 19 %, 56 °C). Workspace mirror: **[jetson_ws/](jetson_ws/README.md)**.

```bash
# On the NX (consolidated catkin ws /home/unitree/autonomous_exploration_zhu/):
scripts/onboard_autonomy_noetic.sh                 # full stack, autonomous explore
scripts/onboard_autonomy_noetic.sh explore=false   # nav only (manual goals)
scripts/onboard_autonomy_noetic.sh slam=fastlio    # FAST-LIO instead of Point-LIO
scripts/onboard_autonomy_noetic.sh stop
```

Full build/deploy recipe + the cross-distro port notes (CFPA2 hexagonal `#ifdef
CFPA2_ROS1`, nvcc 11.4 brace-init fix, sm_87 CUDA gating, trav CNN runtime deps)
live in [jetson_ws/README.md](jetson_ws/README.md) and the gitignored
`scripts/real/.orin_nx_cheatsheet.md` (contains the robot password).

Debug dashboards auto-start:

- Door task: <http://127.0.0.1:8080>
- VLM exploration: <http://localhost:8501>

## Repository Structure

```text
Collab_QRC/
  CLAUDE.md / CLAUDE1.md           Engineering memory (current + archive)
  README.md / setup.sh / env.sh    First-time setup + workspace env
  todo.txt                         Forward-looking research items

  config/
    deployment/                    Hybrid ROS1+ROS2 deployment configs (Phase A)
    dynamic_filter/                Dynamic-LIO + temporal voxel decay
    map_cleanup/                   ERASOR static-map filtering
    slam_backend/                  Swarm-LIO2 / Fast-LIO+SC-PGO selectors
    fastdds_no_shm.xml             FastDDS profile (sim, no shared memory)
    cyclonedds_{ethernet,wifi}.xml CycloneDDS profiles (real robot)

  docker/
    ros1_hybrid_slam/              Swarm-LIO2 + Dynamic-LIO + ERASOR in Noetic + bridge
    ros1_scpgo/                    SC-PGO loop closure (legacy)

  docs/
    claude/                        Per-topic engineering notes (door_task, nav_benchmarks,
                                   real_robot, slam_and_scenes, sim_comparison, …)
    *.md                           Hybrid-SLAM design + integration plans

  external/                        gitignored — fetch_slam_backends.sh pulls Swarm-LIO2,
                                   dynamic_lio, ERASOR, Livox-SDK into here

  jetson_ws/                       Go2 Orin NX onboard catkin workspace mirror (ROS 1 Noetic)
    README.md                      Packages, data flow, build/deploy/run, port gotchas
    src/                           Flat catkin src: Point-LIO, livox, elevation_mapping_cupy,
                                   trav_pipeline_ros1, nav_algo_ros1, cfpa2_* (deployment snapshot)

  scripts/
    launch/      User-invoked sim entry points (nav_test_*, door_demo, vlm_demo, swarm_lio2)
    bench/       Multi-trial PASS-criterion runners + session_reporter
    runtime/     ROS 2 nodes started by launches (fast_lio_tf_adapter, gravity_align_at_init,
                 stuck_watchdog, swarm_lio_tf_adapter, exploration_metrics_logger, …)
    real/        Real-robot entry points (real_autonomy*, record/replay_livox_dataset,
                 connect_ethernet, calibrate_imu, onboard_slam, dry_run_go2)
    debug/       Live-stack observation tools (far_debug_monitor, vlm_debug_web,
                 analyze_lidar_odom_drops, failure_decomposer, …)
    ops/         One-shot dev/ops utilities (far_reset_vgraph, sync_to_main,
                 aic8800-install-fix)
    setup/       SLAM backend fetch + sanity-check scripts

  src/
    go2w/                          Go2 / Go2W platform packages
      go2_description/             URDF + xacro (incl. Mid-360 mount tilt 2026-05-09)
      go2_gazebo_sim/              MJCF scenes + sim launches (Nav2 MPPI / FAR / TARE / RL)
      mujoco_sensor_bridge/        MuJoCo LiDAR / contact / odom bridges
      go2w_nav/ go2w_control/      A*, Hybrid A*, cmd_vel routing (legacy + Nav2 entrypoints)
      go2w_perception/             Cartographer bridge, scan adapter, frontier markers
      go2w_observability/          exploration_metrics_logger (events + summary + stop trigger)
      go2w_safety/                 supervisor_panic, autonomy_enabler, wall-collision checker
      go2w_real_bringup/           Real-robot launches (real_single*, slam.launch.py, real_bringup_core)
      go2w_config/                 Nav2 yaml profiles (default, omni_2d, se2_holonomic) + Cartographer cfg
      go2w_spawn/ unitree_go2w_ros2/  Spawn helpers + Unitree ROS 2 SDK
    collaborative_exploration/
      cfpa2_collaborative_autonomy/  CFPA2 frontier allocator (single + dual coordinator)
      go2_nav_algorithms/            Scan mapper, frontier detection, far_status_adapter
      go2_tare_planner_ros2/         CMU TARE wrapped for ROS 2 (real-robot path)
      slam_backend_adapters/         ROS 2 adapters for Swarm-LIO2 / Dynamic-LIO / ERASOR
      dynamic_scene_filter/          Temporal voxel decay + dynamic obstacle injector
      door_task/                     Dual-robot door task (VLM + analytical barrier)
    vlm_explorer/                  Single-robot VLM-in-the-loop exploration (Phase 1)
    vendor/                        Vendored third-party (not submodules):
      autonomy_stack_go2/            CMU FAR + terrain_analysis + localPlanner
      tare_planner/                  CMU TARE (caochao39, humble-jazzy)
      far_planner/                   FAR V-graph planner (vendored separately)
      fast_lio/                      Fast-LIO2 LiDAR-inertial SLAM (HKU MaRS)
      sc_pgo/                        Scan-context loop closure (post-FLIO2 correction)
      mujoco_ros2_control/           DFKI MuJoCo ros2_control HW interface
      champ/                         CHAMP quadruped locomotion controller
      livox_ros_driver2/ Livox-SDK2/ Mid-360 driver (workspace-local install)
      multirobot_map_merge/          Multi-robot OccupancyGrid merge
      elevation_mapping_gpu_ros2/    GPU elevation mapping (experimental)
      patchwork-plusplus/            Ground-segmentation
      rl_sar/ go2_rl_ws/             RL locomotion experiments (saturation issues — see go2_integration.md)
```

## Architecture (sim Nav2 MPPI path)

```text
MuJoCo physics (500 Hz)
  └─ mujoco_ros2_control  →  PointCloud2, joint_states, /clock
         │
         ├─ Fast-LIO2  →  /Odometry, /cloud_registered{,_body}
         │     └─ fast_lio_tf_adapter  →  /<ns>/odom/nav  +  TF map → base_link
         │
         ├─ octomap_server  →  /<ns>/map (2D projection from /cloud_registered_body)
         │
         ├─ exploration goal source
         │     └─ CFPA2 single_robot_node          →  /<ns>/way_point_coord
         │           └─ cfpa2_to_nav2_bridge       →  /<ns>/goal_pose
         │
         ├─ Nav2 stack (planner_server + controller_server + behavior_server +
         │              bt_navigator + lifecycle_manager)
         │     ├─ SmacPlannerLattice (SE2 holonomic, 0.5 m diff primitives)
         │     ├─ MPPIController DiffDrive (no-strafe, forward-bias critics)
         │     └─ Recovery: BackUp / Spin / Wait
         │
         └─ stuck_watchdog (per-namespace) — outer loop on (v ≈ 0, ω ≈ 0):
                 cancel goal → Nav2 BackUp action → republish goal
```

The real-robot path replaces MuJoCo with `livox_ros_driver2` + Unitree sport API for cmd_vel, and adds `gravity_align_at_init.py` (one-shot, IMU-measured `map → camera_init`) so the SLAM map is gravity-aligned regardless of startup terrain.

## Navigation Backends

Switchable via `nav_backend:=` / `nav=` at launch time.

| Backend | Planner / Controller | Status |
| --- | --- | --- |
| **`nav2_mppi`** | Nav2 SmacPlannerLattice + MPPIController + bt_navigator + lifecycle_manager | **Production** (default since 2026-04-29) — supports `holonomic_profile=off / omni_2d / se2_holonomic` |
| `far` | CMU autonomy stack (terrain_analysis + FAR V-graph + localPlanner + pathFollower) | Escape hatch — kept for benchmark comparison |
| `tare_real` | Real CMU TARE → localPlanner direct (Go2 only, FAR unwired) | Used in `nav_test_go2_tare_real.launch.py` and `real_single_tare_real.launch.py` |
| `astar` | C++ `astar_nav_node` (8-conn A* + pure-pursuit + Stanley + oriented footprint) | Legacy escape hatch; door task pinned here |
| `default` | Python `default_nav.py` (A\* grid + D\* Lite + recovery) | Legacy stable on real robot + door task |

**SE2 holonomic profile (default for Go2W and Go2)** — SmacPlannerLattice with 0.5 m diff lattice primitives + MPPI DiffDrive (no strafe, forward-bias critics, pivot-then-forward execution). Suppresses lateral crab-walk while keeping SE2 planner awareness for narrow geometry.

## SLAM Options

| Backend | Inputs | Output topics | Use |
| --- | --- | --- | --- |
| **Fast-LIO2 + Mid-360** | `/livox/lidar`, `/livox/imu` | `/Odometry`, `/cloud_registered{,_body}` | Default sim + real (Mid-360 onboard) |
| Cartographer 2D + L1 | `/utlidar/transformed_cloud` | `/<ns>/map_prob`, TF map → body | Real robot fallback (Unitree L1 LiDAR) |
| **Swarm-LIO2 hybrid (ROS 1 in Docker)** | `/livox/lidar`, `/livox/imu` (bridged) | `/robot_a/swarm_lio2_raw/{Odometry,cloud_static}` | Phase A additive — used by `nav_test_swarm_lio2_se2.sh` |

**TF / gravity alignment (real Fast-LIO path):** `gravity_align_at_init.py` samples ~1 s of stationary IMU at startup and publishes a latched static `map → camera_init` from the measured gravity vector. The Mid-360 mount tilt (+15°/-2°) lives in `go2_description/xacro/livox_mid360.xacro` as the canonical source; `slam.launch.py` reads it for the `body → base_link` static. Re-calibrating the mount only requires editing the xacro.

**TF / gravity alignment (sim swarm_lio2 path):** `swarm_lio_tf_adapter.py` rebroadcasts swarm_lio2's `quad1/world → quad1_aft_mapped` dynamic into ROS 2 and rewrites cloud frame_ids to play nicely with octomap.

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

# Mark the non-buildable vendored sources before the first colcon invocation.
# COLCON_IGNORE is gitignored (see .gitignore), so each clone must recreate it.
touch src/vendor/autonomy_stack_go2/COLCON_IGNORE  \
      src/vendor/Livox-SDK2/COLCON_IGNORE          \
      src/vendor/sc_pgo/fast_lio_sam/COLCON_IGNORE \
      src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

| Path | Why ignored |
| --- | --- |
| `src/vendor/autonomy_stack_go2/` | CMU upstream, vendored as reference; the active FAR / terrain_analysis / localPlanner builds live in their own packages. |
| `src/vendor/Livox-SDK2/` | Plain CMake library, not a colcon package; built workspace-local by `livox_ros_driver2`'s own install step. |
| `src/vendor/sc_pgo/fast_lio_sam/` | ROS 1 (catkin) — the active SC-PGO loop closure path uses a different sub-tree. |
| `src/mtare_ros1_ws/` | ROS 1 MTARE workspace — kept for reference only. |

YAML + Python are live via symlink-install; C++ requires rebuild.

## Golden Rules

1. Always `use_sim_time: true` for every node in sim. Mixed time domains corrupt maps.
2. Never use stale TF fallback (`tf2::TimePointZero`). Drop the scan instead.
3. Each scan painted exactly once — clear `last_scan_` after processing.
4. Dual-robot TF must be namespaced: remap `/tf` → `/{ns}/tf` for every node.
5. Kill zombie MuJoCo before re-launch (see [debug_notes.md](docs/claude/debug_notes.md)).
6. Benchmark PASS = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. Use ≥10 trials for reliability claims.
7. Real-robot: any Unitree BT pad button press latches a 5 s **supervisor-panic** window — auto `cmd_vel` blocked, FAR disarmed, sticks drive directly. See [real_robot.md](docs/claude/real_robot.md#supervisor-panic-override-any-button-emergency).
8. **MPPI footprint:** with `consider_footprint: false` (the default), MPPI uses `robot_radius + collision_margin_distance` as the rejection zone. Set `consider_footprint: true` + a polygon `footprint:` on both costmaps for narrow corridors. See [CLAUDE.md golden rule 14](CLAUDE.md).
9. **Outer-loop `stuck_watchdog` is required** — Nav2's BT recovery rarely fires because MPPI/DWB self-report success while emitting (v ≈ 0, ω ≈ 0). The per-namespace watchdog catches the silent stalls.
10. **MPPI cruise speed = the PathFollow carrot.** Forward speed emerges from PathFollowCritic chasing `furthest_reached_path_point + offset_from_furthest`, where `furthest_reached` must be the MAX over the trajectory **bundle's endpoints** (`findPathFurthestReachedPoint`). If it's computed as closest-to-robot instead, the carrot pins ~offset cells ahead and MPPI creeps at `~carrot_dist/horizon` (≈0.1 m/s) regardless of path/costmap. Go2W `vx_max=0.40` keeps cruise below the 0.5–0.6 m/s wheel↔legged gait-switch tip zone. See [CLAUDE.md golden rule 25](CLAUDE.md).

## Debugging

```bash
# Topic rates
ros2 topic hz /{ns}/registered_scan
ros2 topic hz /{ns}/odom/nav

# TF lookups (dual-robot must remap)
ros2 run tf2_ros tf2_echo map base_link \
    --ros-args -r /tf:=/{ns}/tf -r /tf_static:=/{ns}/tf_static

# Stale DDS state → clear shared memory
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

# Zombie MuJoCo cleanup (preflight kill helper)
bash scripts/launch/_preflight_kill.sh

# Outdoor Fast-LIO drift analysis on a recorded bag
./scripts/debug/analyze_lidar_odom_drops.py bags/livox_dataset_*  --mask 0,2
```

See [docs/claude/debug_notes.md](docs/claude/debug_notes.md) for cross-cutting gotchas (QoS, zombies, MuJoCo quirks) and [docs/claude/real_robot.md](docs/claude/real_robot.md) for the real-robot bring-up chain.

## Environment

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble
- **Python:** 3.10 (micromamba `cmu_env`)
- **Sim:** MuJoCo 3.6.0 (pip) + DFKI `mujoco_ros2_control`
- **DDS:** FastDDS (sim, `config/fastdds_no_shm.xml`) · CycloneDDS (real robot, `config/cyclonedds_*.xml`)
- **Build:** colcon + ament_cmake / ament_python

## License

Research project. Contact maintainers for licensing.
