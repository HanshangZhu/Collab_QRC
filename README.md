# Collab_QRC — Multi-Robot Autonomous Exploration

Heterogeneous multi-robot autonomy on Unitree **Go2W** (wheeled-legged) + **Go2** (walking) quadrupeds. ROS 2 Humble + **MuJoCo** simulation + native **ROS 1 Noetic** deployment on the real Go2's Jetson Orin NX. Primary experiment: centralised vs decentralised frontier allocation under inter-robot comms dropout, evaluated on a real SLAM-reconstructed ops2 building corridor.

Detailed technical reference: **[CLAUDE.md](CLAUDE.md)** — Full HIL setup: **[HOW_TO_RUN_HIL.md](HOW_TO_RUN_HIL.md)** — Onboard deployment: **[jetson_ws/README.md](jetson_ws/README.md)**

## Key Results

| System | Metric | Measured Value |
|---|---|---|
| CFPA2 C++ port | Tick p95, Orin Nano (vs 1376 ms Python) | **1.1 ms (~1250×)** |
| CUDA-MPPI | Kernel chain, Orin Nano | **~1.15 ms GPU / ~12 ms CPU (~10×)** |
| Full autonomy stack | Orin NX CPU / GPU / temp | **<33% / <19% / 56°C** |
| Single-robot exploration | Ops2 building corridor | **Bidirectional ±35 m validated** |
| Benchmark | Coordination × comms dropout | **Centralised vs decentralised, 0/30/50/80%** |

## System Architecture

```
MuJoCo ops2 (500 Hz)  OR  Mid-360 LiDAR (real / HIL)
  └─ mujoco_sensor_bridge  /  livox_ros_driver2
        │
        ├─ Fast-LIO2 (desktop sim, SIM_GT_ODOM=1 for z-drift guard)
        ├─ Point-LIO (Orin NX native ROS 1 Noetic)
        └─ fast_lio_tf_adapter  →  /<ns>/odom/nav  +  TF map→base_link
        │
        ├─ elevation_mapping_cupy  →  /elevation_map_raw
        │   └─ grid_map_filters (CNN + analytical ramp_safe fusion)
        │       └─ grid_map_to_occupancy_grid  →  /<ns>/traversability_grid
        │
        ├─ CFPA2 C++ node  (centralised or decentralised)
        │   └─ cfpa2_to_nav2_bridge  →  /<ns>/goal_pose
        │
        └─ Nav2  (SmacPlannerLattice SE2 + CUDA-MPPIController)
             └─ stuck_watchdog  (outer-loop stall recovery)
```

## Prerequisites & Build

```bash
# Requires: Ubuntu 22.04, ROS 2 Humble, micromamba cmu_env, MuJoCo 3.6.0 (pip), CUDA

micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# One-time per clone — mark non-buildable vendored sources:
touch src/vendor/autonomy_stack_go2/COLCON_IGNORE \
      src/vendor/Livox-SDK2/COLCON_IGNORE \
      src/vendor/sc_pgo/fast_lio_sam/COLCON_IGNORE \
      src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

source install/setup.bash
```

YAML + Python changes are live via symlink-install. C++ changes require `colcon build --packages-select <pkg>`.

## Quick Start

### Desktop sim — single-robot ops2 (Go2 walking)

The ops2 scene is a real SLAM-reconstructed building (80×32 m corridor). Single-robot uses **Go2 (walking)** only — the full mesh convex hull at this scale is collision-unstable for Go2W. The dual-robot mixed scene uses 44 hand-traced wall boxes instead, which is why Go2W works there but not here.

```bash
./scripts/launch/nav_test_slam_ops2_v4_go2.sh              # GUI + RViz2
./scripts/launch/nav_test_slam_ops2_v4_go2.sh gui:=false rviz:=false  # headless
```

### Desktop sim — dual-robot ops2 (heterogeneous Go2W + Go2)

```bash
./scripts/launch/nav_test_slam_ops2_v4_mixed.sh             # centralised (default)
COORDINATION_MODE=decentralised \
  ./scripts/launch/nav_test_slam_ops2_v4_mixed.sh           # decentralised
```

### Dropout benchmark (centralised vs decentralised, multi-trial)

```bash
# Default: 2 modes × 4 dropout rates × 10 trials × 600 sim-s, headless
./scripts/bench/benchmark_decentralisation_dropout.sh

# Override any defaults via env:
NUM_TRIALS=5 DURATION_SEC=300 DROPOUTS="0.0 0.5" \
  ./scripts/bench/benchmark_decentralisation_dropout.sh

# Aggregate results + generate coverage-vs-dropout plots:
python3 scripts/bench/summarize_dropout_benchmark.py /tmp/dropout_bench/<ts>/
```

### Hardware-in-the-Loop bench (Orin NX)

The laptop simulates the physical world (MuJoCo ops2-v4 + fake Mid-360 + CHAMP locomotion); the Orin NX runs the full ROS 1 Noetic autonomy stack (Point-LIO + traversability CNN + CUDA-MPPI + CFPA2 C++). Sensors and cmd_vel cross via a C++ UDP relay.

```bash
./scripts/launch/hil_orin_nx.sh up                          # start both sides
./scripts/launch/hil_orin_nx.sh up lidar_range=2.5 max_vel=0.4
./scripts/launch/hil_orin_nx.sh stop
```

First-time NIC configuration and prerequisites: **[HOW_TO_RUN_HIL.md](HOW_TO_RUN_HIL.md)**

## Real-Robot Onboard (Orin NX)

The complete autonomy stack runs natively on the Go2's Jetson Orin NX 16 GB in ROS 1 Noetic — no ros1_bridge in the data path. SLAM (Point-LIO) + traversability (elevation_mapping_cupy + CNN) + nav (SmacLattice + CUDA-MPPI via move_base) + frontier exploration (CFPA2 C++).

```bash
# On the Orin NX (192.168.123.18):
scripts/real/onboard_autonomy_noetic.sh               # autonomous exploration
scripts/real/onboard_autonomy_noetic.sh explore=false   # nav only (manual goals)
scripts/real/onboard_autonomy_noetic.sh stop

# From laptop — deploy changes:
./scripts/real/deploy_noetic_to_jetson.sh
```

Build recipe, cross-distro port notes (nvcc brace-init fix, sm_87 CUDA gating, trav CNN runtime deps): **[jetson_ws/README.md](jetson_ws/README.md)**

## Repository Layout

```
Collab_QRC/
  CLAUDE.md / CLAUDE1.md           Technical reference + development history archive
  README.md / HOW_TO_RUN_HIL.md    This file + HIL bench setup guide
  jetson_ws/                       Orin NX catkin workspace mirror (ROS 1 Noetic)

  scripts/
    launch/     Sim entry points (nav_test_slam_ops2_v4_*, hil_orin_nx, …)
    bench/      Benchmark runners + summarizers
    runtime/    ROS 2 nodes (cfpa2_to_nav2_bridge, comms_dropout_relay,
                stuck_watchdog, fast_lio_tf_adapter, exploration_metrics_logger, …)
    real/       Real-robot tooling (onboard_autonomy_noetic, deploy scripts, …)
    debug/      Live observation tools

  src/
    go2w/
      go2_gazebo_sim/               MJCF scenes + sim launch files
      go2w_config/                  Nav2 yaml profiles + behavior trees
      go2w_control/                 Hybrid cmd_vel router (wheel/legged mux)
      go2w_observability/           exploration_metrics_logger + coverage_util
      hil_udp_relay/                C++ UDP relay (HIL sensor/cmd_vel bridge)
      mujoco_sensor_bridge/         MuJoCo LiDAR / contact / odom bridges
    collaborative_exploration/
      cfpa2_collaborative_autonomy/ CFPA2 C++ frontier allocator
                                    (core/ + ros2/ + ros1/ adapter layers)
      cfpa2_peer_coordination/      Decentralised peer blocked-frontier relay
      trav_cost_filters/            ETH traversability pipeline
                                    (filter_chain_runner + grid_map_to_occ)
      slam_backend_adapters/        Fast-LIO ↔ Nav2 TF adapter
    vendor/
      fast_lio/                     Fast-LIO2 SLAM
      nav2_mppi_controller_cuda/    Patched Nav2 MPPI (ICudaBackend injection)
      nav_algo_ros1/                Nav2 SmacLattice + MPPI ported to ROS 1 + CUDA
      elevation_mapping_cupy/       ETH GPU elevation mapping
      mujoco_ros2_control/          DFKI MuJoCo ros2_control HW interface
      champ/                        CHAMP quadruped locomotion controller
      livox_ros_driver2/            Mid-360 LiDAR driver

  docs/
    claude/          Per-topic engineering notes (indexed in CLAUDE.md §13)
    superpowers/     Design specs + implementation plans

  config/
    fastdds_no_shm.xml             FastDDS profile (sim, shared memory disabled)
    cyclonedds_*.xml               CycloneDDS profiles (real robot)
```

## Environment

| | Value |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Python | 3.10 (micromamba `cmu_env`) |
| Sim | MuJoCo 3.6.0 (pip) + DFKI `mujoco_ros2_control` |
| DDS | FastDDS (sim) · CycloneDDS (real robot) |
| Real robot | Jetson Orin NX 16 GB, Ubuntu 20.04 aarch64, ROS 1 Noetic, CUDA 11.4 |
| SLAM (sim / desktop) | Fast-LIO2 |
| SLAM (real / HIL) | Point-LIO (native ROS 1 Noetic) |

## License

Research project. Contact maintainers for licensing.
