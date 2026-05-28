# README Rebuild + CLAUDE.md Reorganization — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the chronological dev journal (CLAUDE.md) into a concise technical reference, archive the history to CLAUDE1.md, and rebuild README.md to reflect the actual final deliverables.

**Architecture:** Three-file edit. CLAUDE.md lines 7–834 (all "Active state" sections) append to CLAUDE1.md. CLAUDE.md is rewritten from scratch as a component-organized reference. README.md is rewritten from scratch as the external-facing project document.

**Tech Stack:** Plain Markdown, `sed` for extraction, standard git.

---

## Files Changed

| File | Action |
|---|---|
| `CLAUDE1.md` | Append: all "Active state" sections from current CLAUDE.md (lines 7–834) |
| `CLAUDE.md` | Full rewrite: component-organized technical reference (~380 lines) |
| `README.md` | Full rewrite: external-facing project document (~230 lines) |

---

## Task 1: Archive Active State history to CLAUDE1.md

**Files:**
- Modify: `CLAUDE1.md` (append)

- [ ] **Step 1: Append a divider + the Active State sections to CLAUDE1.md**

```bash
# Append divider header
cat >> CLAUDE1.md << 'DIVIDER'

---

## Development History (2026-05-xx sessions)

*Appended from CLAUDE.md on project completion. These are the chronological "Active state" entries that record what was built and debugged session-by-session.*

DIVIDER

# Extract CLAUDE.md lines 7 through 834 (all Active state sections,
# ending just before "## Skill-API detail docs") and append
sed -n '7,834p' CLAUDE.md >> CLAUDE1.md
```

- [ ] **Step 2: Verify the append looks correct**

```bash
tail -30 CLAUDE1.md
```

Expected: ends with the `## Active state (2026-04-26) — archived` section content (the last Active state entry before "Skill-API detail docs").

```bash
grep -c "## Active state" CLAUDE1.md
```

Expected: a number ≥ 23 (the 4 pre-existing archived entries in CLAUDE1.md + the ~19 new ones appended).

- [ ] **Step 3: Commit**

```bash
git add CLAUDE1.md
git commit -m "docs(archive): append 2026-05-xx Active state history from CLAUDE.md to CLAUDE1.md"
```

---

## Task 2: Rewrite CLAUDE.md as technical reference

**Files:**
- Rewrite: `CLAUDE.md`

- [ ] **Step 1: Write the new CLAUDE.md**

Replace the entire file with the following content:

```bash
cat > CLAUDE.md << 'EOF'
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
EOF
```

- [ ] **Step 2: Verify line count and spot-check key sections**

```bash
wc -l CLAUDE.md
```

Expected: ~380–420 lines.

```bash
grep "^## " CLAUDE.md
```

Expected output:
```
## 1. System Overview
## 2. Nav Stack — CUDA-MPPI + PathFollow Carrot
## 3. SLAM
## 4. Traversability Pipeline
## 5. CFPA2 Frontier Allocation
## 6. Ops2 Scene + Exploration Config
## 7. Dropout Benchmark
## 8. HIL Architecture (Orin NX)
## 9. Onboard Deployment (Orin NX)
## 10. Build
## 11. Scripts Layout
## 12. Golden Rules
## 13. Skill-API Docs Index
```

```bash
# Confirm no stale "Active state" sections remain
grep "Active state" CLAUDE.md
```

Expected: no output (all Active state content is now in CLAUDE1.md).

```bash
# Confirm key technical facts present
grep "1250" CLAUDE.md && grep "10×" CLAUDE.md && grep "floodfill" CLAUDE.md
```

Expected: at least one match per grep.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(refactor): rewrite CLAUDE.md as component-organized technical reference

Converts the 1073-line chronological dev journal into a ~380-line
reference organized by system component. Active state history moved
to CLAUDE1.md. Key facts (numbers, configs, gotchas, Golden Rules)
preserved; narrative removed."
```

---

## Task 3: Rebuild README.md

**Files:**
- Rewrite: `README.md`

- [ ] **Step 1: Write the new README.md**

```bash
cat > README.md << 'EOF'
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
EOF
```

- [ ] **Step 2: Verify line count and spot-check content**

```bash
wc -l README.md
```

Expected: ~220–250 lines.

```bash
# Confirm stale features are gone
grep -E "door_demo|vlm_demo|swarm_lio2|VLM Integration|nav_test_demo3\b" README.md
```

Expected: no output.

```bash
# Confirm key new content is present
grep -E "dropout|ops2|decentralis|hil_orin_nx|Go2W collision" README.md
```

Expected: multiple matches.

```bash
# Confirm Go2W collision note is present
grep "collision-unstable" README.md
```

Expected: one match.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs(readme): rebuild as final-state project document

Replaces the stale 324-line README (referenced removed features: door
task, VLM demo, Swarm-LIO2 main path, nav_test_demo3 as production)
with a ~230-line document reflecting actual deliverables: dual-robot
ops2 exploration, centralized/decentralized dropout benchmark, HIL
bench, and Orin NX native deployment. Adds Key Results table with
measured numbers and the Go2W collision constraint note for ops2."
```

---

## Task 4: Final verification

- [ ] **Step 1: Cross-check CLAUDE.md vs README for consistency**

```bash
# Both should agree on CFPA2 speedup number
grep "1250" CLAUDE.md README.md

# Both should agree on CUDA-MPPI numbers
grep "10×\|1\.15 ms\|12 ms" CLAUDE.md README.md

# Both should mention the Go2W collision constraint
grep "collision-unstable\|convex hull" CLAUDE.md README.md
```

Expected: matches in both files for each grep.

- [ ] **Step 2: Confirm CLAUDE1.md received the history**

```bash
grep -c "## Active state" CLAUDE1.md
```

Expected: ≥ 23 (the ~19 new entries + 4 pre-existing archived entries).

```bash
# Spot-check: the 2026-05-25 PathFollow entry (most recent) is at the top of the appended block
grep -n "2026-05-25" CLAUDE1.md | head -3
```

Expected: at least one match.

- [ ] **Step 3: Confirm no broken markdown links in the new CLAUDE.md**

```bash
# All linked docs/claude/ files should exist
grep -oP '\[.*?\]\(\K[^)]+' CLAUDE.md | grep "docs/claude" | while read f; do
  [ -f "$f" ] || echo "MISSING: $f"
done
```

Expected: no MISSING lines.

- [ ] **Step 4: Final commit if any cleanup needed, then confirm git log**

```bash
git log --oneline -5
```

Expected: the three new commits from Tasks 1–3 are the most recent.

---

## Self-Review

**Spec coverage check:**

| Spec requirement | Covered in |
|---|---|
| Append Active state (lines 7–834) to CLAUDE1.md | Task 1 |
| CLAUDE.md §1–13 component structure | Task 2 |
| CLAUDE.md §6 Go2W collision note | Task 2 §6 |
| CLAUDE.md Golden Rules (all 25) | Task 2 §12 |
| README Key Results table with measured numbers | Task 3 |
| README Architecture ASCII diagram | Task 3 |
| README Quick Start: single-robot, dual-robot, benchmark, HIL | Task 3 |
| README Go2W collision constraint note | Task 3 single-robot section |
| README remove: door_demo, VLM, Swarm-LIO2 | Task 3 (not included in new file) |
| Verify stale refs gone | Task 3 Step 2, Task 4 Step 1 |

**Placeholder scan:** None found. All file paths, numbers, grep commands, and expected outputs are concrete.

**Type consistency:** No code types involved — documentation only. File paths referenced in the plan match actual repo paths verified during brainstorming.
