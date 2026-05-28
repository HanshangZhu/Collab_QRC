# CLAUDE.md — Collab_QRC Technical Reference

Multi-robot autonomous exploration with Unitree Go2 / Go2W quadrupeds. ROS 2 Humble + MuJoCo (primary sim) + native ROS 1 Noetic on the real Go2's Orin NX. Development history archived in [CLAUDE1.md](CLAUDE1.md).

## 1. System Overview

**Final tech stack:**
- SLAM: Point-LIO (Orin NX production), Fast-LIO2 (sim / desktop)
- Traversability: ETH elevation_mapping_cupy → grid_map_filters (CNN + analytical fusion) → OccupancyGrid
- Nav: Nav2 SmacPlannerLattice (SE2 holonomic) + CUDA-MPPIController
- Frontier: CFPA2 C++ (centralised or decentralised coordination)
- Primary scene: ops2 SLAM-reconstructed building (80×32 m corridor)
- Robots: Go2W (wheeled-legged, robot_a) + Go2 (walking, robot_b), heterogeneous pair

**Key verified numbers (all measured, not projected):**

| Metric | Value |
|---|---|
| CFPA2 C++ tick p95 (Orin Nano) | 1.1 ms — was 1376 ms Python (~1250×) |
| CUDA-MPPI kernel chain (Orin Nano, measured) | ~1.15 ms GPU vs ~12 ms CPU (~10×) |
| Full autonomy stack (Orin NX) | CPU <33%, GPU <19%, 56°C |
| Single-robot ops2 corridor | Bidirectional ±35 m validated |

**CUDA-MPPI honest framing:** ~10× is the kernel chain only. At ~12 ms/cycle the CPU path is ~24% of the 50 ms @ 20 Hz budget — both CPU and GPU MPPI sustain 20 Hz. CUDA-MPPI's real value is as a CPU-offload (moves ~12 ms off the contended CPU onto the otherwise-idle GPU <19%), freeing cores for Point-LIO and CFPA2. The genuine real-time rescue was CFPA2 C++ (~1250×, the one that crossed the 500 ms budget).

## 2. Nav Stack — CUDA-MPPI + PathFollow Carrot

**The PathFollow carrot (Golden Rule 25):**
MPPI cruise speed emerges from `PathFollowCritic` chasing a carrot at `furthest_reached_path_point + offset_from_furthest`. `furthest_reached` must be the **MAX** over the trajectory bundle's endpoints, computed **AFTER** the integrate kernel. Computing it as closest-to-robot (before the kernel) pins the carrot at a fixed offset → steady creep at `carrot_dist / (T·dt) ≈ 0.1 m/s` regardless of path length or costmap. Fixed in [`cuda_backend.cu`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/cuda_backend.cu).

**Speed cap (Go2W):**
`vx_max: 0.40` in [`nav2_go2w_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml). Keeps cruise below the 0.5–0.6 m/s wheel↔legged gait-switch tip zone.

**Wheel-engage gate** ([`go2w_hybrid_motion.yaml`](src/go2w/go2w_config/config/control/go2w_hybrid_motion.yaml)):
`wheel_linear_threshold: 0.10`, `wheel_engage_sustain_sec: 0.25`.

**Rebuild after `.cu` changes:**
```bash
colcon build --symlink-install --packages-select nav2_mppi_controller_cuda_plugin
```
YAML changes are live via symlink-install (no rebuild needed).

**MPPI timing probe (off by default):**
Set `MPPI_TIME_CSV=/path/log.csv` to record wall-time of each `optimize()` call for CPU vs GPU A/B comparison.

## 3. SLAM

| | Point-LIO | Fast-LIO2 |
|---|---|---|
| Use | Orin NX production (real + HIL) | Desktop sim |
| Config | `jetson_ws/src/point_lio/config/mid360_go2_real.yaml` | `src/vendor/fast_lio/config/mid360.yaml` |
| Odometry | `/<ns>/Odometry` | `/<ns>/Odometry` |

**`fast_lio_tf_adapter.py`** ([`scripts/runtime/`](scripts/runtime/)): sole owner of `odom → base_link` TF and `/<ns>/odom/nav`. `mujoco_odom_bridge.publish_tf` must stay `false` to avoid competing on the same TF link.

**`SIM_GT_ODOM=1`** (default in ops2 desktop launchers): drives `/<ns>/odom/nav` from MuJoCo ground truth so standalone Fast-LIO z-drift doesn't corrupt CFPA2/Nav2. The traversability grid still uses real perception (body-frame cloud + GT TF). Real robot / HIL keep gated Fast-LIO.

## 4. Traversability Pipeline

```
elevation_mapping_cupy  →  /elevation_map_raw  (elevation, variance, traversability)
filter_chain_runner      →  /elevation_map_filtered  (+ slope, step_height, wall_cost_dilated, trav_fused)
grid_map_to_occupancy_grid  →  /<ns>/traversability_grid  (OccupancyGrid, Nav2 cost convention)
```

**CNN backend:** pure cupy reimplementation (65 lines, float32) of the 3-branch dilated conv stack + 1×1 fusion + `exp(-|·|)`. Auto-selected when `cudaDeviceProperties.major >= 10` (Blackwell/RTX 5090). On other GPUs: torch backend. On Blackwell torch fails (`CUDA_ERROR_NO_BINARY_FOR_GPU`) — set `ELEVATION_MAPPING_FORCE_CUPY=1`.

**Key config files:**
- [`elevation_mapping.yaml`](src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml): `max_height_range: 1.7` m (keeps bridge/awning tops out), `min_valid_distance: 0.40` m
- [`grid_map_filters.yaml`](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml): `wall_cost_dilated` window_size: **3** (not 5) → ±0.10 m dilation instead of ±0.20 m

**Visited corridor** ([`grid_map_to_occupancy_grid.py`](src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py)): accumulates the swept capsule between consecutive robot poses and forces those cells FREE permanently. Keeps the reachable BFS component connected through the Mid-360 geometric blind disk behind the robot.

**Rebuild required** (ament_cmake_python installs as copy, not symlink):
```bash
colcon build --symlink-install --packages-select trav_cost_filters
```

## 5. CFPA2 Frontier Allocation

**Architecture — hexagonal isolation:**
- Algorithm body: `cfpa2::core::*` — zero ROS calls, communicates through abstract interfaces
- ROS 2 adapters: `include/cfpa2_collaborative_autonomy/ros2/` (header-only)
- ROS 1 adapters: `include/cfpa2_collaborative_autonomy/ros1/` (Orin NX Noetic port)
- Select binary via launch arg: `cfpa2_executable_suffix:=_cpp`

**Coordination modes:**
- `coordination_mode:=centralised` — joint allocator scores all (goal_a, goal_b) pairs, picks max with multiplicative overlap penalty: `joint = (u_a + u_b) × clamp(1 − λ·overlap, 0, 1)`
- `coordination_mode:=decentralised` — each robot runs its own CFPA2; peer blocked-frontiers relayed via `/<ns>/cfpa2_peer_coordination/blocked_frontiers`

**Critical param** ([`cfpa2_single_robot_ops2.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot_ops2.yaml)):
- `cfpa2_ig_mode: "local"` — **must be "local", NOT "floodfill"**. The floodfill single-call path is an unimplemented stub returning 0.0 → all frontiers get utility −1e18 → robot freezes.

**Other key params for ops2:**
- `allow_unknown: false`
- `cfpa2_frontier_obstacle_clearance_m: 0.20`
- `inflation_radius: 0.16` (cost-0 lane 0.68 m > Go2's 0.64 m turning envelope)

**Rebuild:**
```bash
colcon build --symlink-install --packages-select cfpa2_collaborative_autonomy
```

## 6. Ops2 Scene + Exploration Config

**Scene files:**
- Single-robot: [`slam_ops2_v4_go2_handwalls.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_handwalls.xml)
- Dual-robot: [`slam_ops2_v4_mixed_handwalls.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_mixed_handwalls.xml)

**Why single-robot ops2 uses Go2 (not Go2W):** The ops2 visual mesh (80×32 m, 982k verts) is collision-inactive (`contype=0 conaffinity=0`). MuJoCo collides `<geom type="mesh">` via the convex hull of the whole mesh — at this scale the hull engulfs the Go2W's standing volume, making it collision-unstable. Go2's smaller footprint is stable. The dual-robot mixed launch uses 44 hand-traced wall boxes for collision instead of the whole-mesh hull, so Go2W works there.

**Key exploration config in production:**
- 3.0 m forced-free disk (`robot_seed_radius_m: 3.0`) — covers Mid-360 geometric blind zone (~3.25 m at −7° V-FOV start angle)
- Visited corridor stamp (`visited_corridor_enabled: true`, radius 0.45 m) — prevents trail fragmentation in the Mid-360 blind zone behind the robot
- Extent-seek strategy — turns around once ±x extreme is physically reached
- Inflation 0.16 m — keeps cost-0 lane wider than Go2's 0.64 m turning envelope
- IG-override disabled in ops2 overlay — extent-seek alone handles the −x branch

## 7. Dropout Benchmark

**Entry point:** [`scripts/bench/benchmark_decentralisation_dropout.sh`](scripts/bench/benchmark_decentralisation_dropout.sh)

**Key tunables (env vars, with defaults):**
```
MODES="centralised decentralised"   # coordination modes to test
DROPOUTS="0.0 0.3 0.5 0.8"         # inter-robot comms drop rates
NUM_TRIALS=10                       # trials per condition
DURATION_SEC=600                    # sim-seconds per trial
SCENE_AREA_M2=384.0                 # for coverage normalization
CFPA2_SUFFIX=_cpp                   # always use C++ binary
MAX_RETRIES=3                       # max re-runs for SLAM-diverged trials
DIVERGENCE_BOUND_M=60               # odom jump threshold for divergence detection
```

**Output layout:**
```
/tmp/dropout_bench/<ts>/<mode>/d<pct>/trial_<n>/metrics.csv
                                                launch.log
                                                divergence.json
```

**Summarize + plot:**
```bash
python3 scripts/bench/summarize_dropout_benchmark.py /tmp/dropout_bench/<ts>/
# → summary.json + coverage_vs_dropout.png (per condition, mode × dropout)
```

**Support nodes:**
- [`scripts/runtime/comms_dropout_relay.py`](scripts/runtime/comms_dropout_relay.py): drops inter-robot frontier messages at the specified rate (seeded, reproducible)
- [`scripts/runtime/odom_divergence_monitor.py`](scripts/runtime/odom_divergence_monitor.py): flags trials where Fast-LIO diverges (pose > DIVERGENCE_BOUND_M from spawn) → trial auto-retried

## 8. HIL Architecture (Orin NX)

```
  ┌─ LAPTOP ─────────────────────────────┐   UDP    ┌─ Orin NX (192.168.123.18) ──────┐
  │ MuJoCo ops2-v4 + fake Mid-360        │ ───────▶ │ Point-LIO SLAM                  │
  │ CHAMP (consumes /robot/cmd_vel)      │  9001-2  │ elevation_mapping_cupy + CNN    │
  │ /livox/lidar port 9001               │          │ move_base (SmacLattice+MPPI)    │
  │ /livox/imu   port 9002               │ ◀─────── │ CFPA2 C++ exploration           │
  │ RViz2                      cmd_vel   │  9003-5  │ onboard_autonomy_noetic.sh      │
  └──────────────────────────────────────┘   UDP    └─────────────────────────────────┘
```

**Entry point:** [`scripts/launch/hil_orin_nx.sh`](scripts/launch/hil_orin_nx.sh)
```bash
./scripts/launch/hil_orin_nx.sh up                          # start both sides
./scripts/launch/hil_orin_nx.sh up lidar_range=2.5 max_vel=0.4
./scripts/launch/hil_orin_nx.sh stop
```
Full NIC setup and one-time prerequisites: **[HOW_TO_RUN_HIL.md](HOW_TO_RUN_HIL.md)**

**Lifecycle gating:** `platform_ready` latched topic gates Point-LIO IMU init. The Go2 must be fully stood up and still before Point-LIO starts (IMU init during stand-up → wrong gravity → z drifts to +5.8 m). `stand_up_slowly.py` holds open 20 s then publishes `platform_ready`; `run_jetson_hil.sh` blocks on it.

**Verified resource use (Orin NX, full stack live):** RAM 6.6/15.4 GB, CPU <33%, GPU <19%, 56°C, 6.6 W.

## 9. Onboard Deployment (Orin NX)

**Catkin workspace:** `/home/unitree/autonomous_exploration_zhu/` on the NX. Mirrored to [`jetson_ws/`](jetson_ws/).

**Entry point:**
```bash
# On the Orin NX (192.168.123.18):
scripts/real/onboard_autonomy_noetic.sh              # autonomous exploration
scripts/real/onboard_autonomy_noetic.sh explore=false  # nav only (manual goals via RViz2)
scripts/real/onboard_autonomy_noetic.sh slam=fastlio   # swap SLAM
scripts/real/onboard_autonomy_noetic.sh stop
```

**Deploy from laptop:**
```bash
./scripts/real/deploy_noetic_to_jetson.sh
```

**Key port gotchas (full detail in [`jetson_ws/README.md`](jetson_ws/README.md)):**
- nvcc 11.4 brace-init parse bug: use copy-init `= rclcpp::get_logger(…)` in 6 nav_algo_core headers
- sm_89 unsupported by CUDA 11.4: gencode gated to `CUDA >= 11.8`
- cupy 12 required (cupy 14 conflicts with system cv_bridge numpy 1 ABI); `float16.cuh` shim needed
- scipy ≥ 1.13 must shadow system scipy 1.3.3: `pip --user 'scipy>=1.13'`

## 10. Build

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Mark non-buildable vendored sources (must redo after each fresh clone):
touch src/vendor/autonomy_stack_go2/COLCON_IGNORE \
      src/vendor/Livox-SDK2/COLCON_IGNORE \
      src/vendor/sc_pgo/fast_lio_sam/COLCON_IGNORE \
      src/mtare_ros1_ws/COLCON_IGNORE

# Full build
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

YAML + Python are live via symlink-install. Packages that need a rebuild after source edits:
- `nav2_mppi_controller_cuda_plugin` — after `.cu` changes
- `trav_cost_filters` — Python copy-install (not symlink)
- `cfpa2_collaborative_autonomy` — C++ binary

## 11. Scripts Layout

```
scripts/
├── launch/    user-invoked sim entry points (nav_test_slam_ops2_v4_*, hil_orin_nx, …)
├── bench/     multi-trial benchmark runners + summarizers
├── runtime/   ROS 2 nodes started by launch files (cfpa2_to_nav2_bridge, comms_dropout_relay,
│              stuck_watchdog, fast_lio_tf_adapter, exploration_metrics_logger, …)
├── real/      real-robot tooling (onboard_autonomy_noetic, deploy_noetic_to_jetson, …)
├── debug/     observe-a-running-sim tools
└── ops/       one-shot dev/ops utilities
```

## 12. Golden Rules

1. Always `use_sim_time: true` for all nodes in sim. Mixed time domains corrupt maps.
2. Never use stale TF fallback (`tf2::TimePointZero`). Drop the scan on TF failure.
3. Each scan painted exactly once — clear `last_scan_` after processing.
4. Dual-robot TF must be namespaced: remap `/tf` → `/{ns}/tf` for every node.
5. DDS config matters. `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS.
6. Verify with `ros2 topic hz` after changing sensor rates. Xacro changes require model re-spawn.
7. Kill zombie MuJoCo before re-launch (see [debug_notes.md](docs/claude/debug_notes.md)).
8. Benchmark PASS = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. Use ≥10 trials for reliability claims.
9. Real-robot: any Unitree BT pad button latches a 5 s supervisor-panic window — auto `cmd_vel` blocked, FAR disarmed. See [real_robot.md](docs/claude/real_robot.md).
10. Any node doing TF lookup in dual-robot setup MUST have `tf_remaps`. Without `("/tf", f"/{ns}/tf")`, TF buffer subscribes the global `/tf` (empty in our namespaced setup) and every lookup silently fails — no error, just stale data.
11. CMU `vehicle` frame is NOT in our SLAM tree by default. Mixed launch bridges `base_link → vehicle` per namespace.
12. Multi-layer safety stacks deadlock easily. Always provide a max-hold/timeout escape on each stateful safety layer.
13. `peer_pose_stale_sec` must be generous in sim (5.0 s default). Tighten on real robot only after verifying timestamp alignment.
14. MPPI footprint: set `consider_footprint: true` + polygon on both costmaps for narrow corridors. Drop `collision_margin_distance` to 0.03 m. The "throws at configure" rumour was a yaml-parse bug, not a platform bug.
15. Footprint changes ripple through CFPA2 frontier filters. CFPA2 has no parameter callback — yaml edits only take effect after node restart.
16. Sim/real share the same TF + odom path via `fast_lio_tf_adapter`. `mujoco_odom_bridge.publish_tf` must stay `false`.
17. Don't blame the architecture before grepping the upstream. Vendored source bugs hide at the lowest layer.
18. Outer-loop stuck recovery is needed — MPPI/DWB rarely report failure; they emit (v≈0, ω≈0) and self-report happy. `stuck_watchdog.py` catches silent stalls.
19. `SmacPlanner2D` is XY-only. Use `SmacPlannerLattice` for heading-dependent footprint-fit maneuvers.
20. Missing file errors under `install/.../share/<pkg>/...` = stale install artifacts. Rebuild the package.
21. Don't hardcode absolute controller_manager-namespaced topic paths. Use relative defaults; verify `ros2 topic info <topic> -v` shows `publisher_count > 0` before assuming the node is wired.
22. CFPA2 algorithm body is ROS-independent (hexagonal isolation). New algorithm code goes through `core::IClock / ILogger / IGoalPublisher / IVisualizer` interfaces — never call `get_clock()->now()` or `RCLCPP_*` from algorithm body.
23. ROS 2 algorithm code can be ported byte-for-byte to ROS 1 via a thin `compat.hpp` shim — NOT a rewrite. See [`nav_algo_core/include/nav_algo_core/compat.hpp`](src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/compat.hpp).
24. GPU acceleration goes through `ICudaBackend` injection — algorithm package stays CUDA-free. Concrete CUDA impl lives in a separate package; `use_cuda: true/false` yaml flag; CPU fallback is silent.
25. **MPPI cruise speed = the PathFollow carrot.** `furthest_reached_path_point` must be the MAX over the trajectory bundle's endpoints (computed AFTER the integrate kernel), NOT the path index closest to the robot's current pose. The latter pins the carrot and produces steady-state creep at `≈carrot_dist/horizon` regardless of path length or costmap. See §2 and [`cuda_backend.cu`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/cuda_backend.cu).

## 13. Skill-API Docs Index

| Topic | Doc |
|---|---|
| Nav stack benchmarking, config A, iteration logs | [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) |
| Fast-LIO2 / Cartographer A/B, LiDAR options, demo scenes | [docs/claude/slam_and_scenes.md](docs/claude/slam_and_scenes.md) |
| Cross-cutting debugging gotchas (QoS, zombies, MuJoCo quirks) | [docs/claude/debug_notes.md](docs/claude/debug_notes.md) |
| Gazebo vs MuJoCo — why stack works in Gazebo, MuJoCo matches real life | [docs/claude/sim_comparison.md](docs/claude/sim_comparison.md) |
| Real Go2W / Go2 — connect, SLAM A/B, Mid-360 calib, bug chain | [docs/claude/real_robot.md](docs/claude/real_robot.md) |
| Go2 integration — Menagerie MJCF, CHAMP, TARE, RL scaffold | [docs/claude/go2_integration.md](docs/claude/go2_integration.md) |
| Nav2 MPPI migration journey (2026-04-29) | [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md) |
| Noetic FAST-LIO2 onboard (2026-05-11) | [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md) |
| Noetic port checklist (CFPA2 ROS 1 adapter guide, ~5 h estimate) | [docs/claude/noetic_port_checklist.md](docs/claude/noetic_port_checklist.md) |
| Ops2 bidirectional exploration (run-by-run + final config) | [docs/claude/ops2_bidirectional_exploration_journey.md](docs/claude/ops2_bidirectional_exploration_journey.md) |
| Orin NX HIL design rationale + data-flow diagram | [docs/claude/orin_nx_hil_design.md](docs/claude/orin_nx_hil_design.md) |
| ETH elevation mapping design reference | [docs/claude/eth_elevation_mapping_design.md](docs/claude/eth_elevation_mapping_design.md) |
| Decentralised CFPA2 coordination notes | [docs/claude/decentralisation.md](docs/claude/decentralisation.md) |
| Dropout benchmark spec | [docs/superpowers/specs/2026-05-24-decentralisation-dropout-benchmark-design.md](docs/superpowers/specs/2026-05-24-decentralisation-dropout-benchmark-design.md) |
