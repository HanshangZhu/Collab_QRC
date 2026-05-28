# Spec: README rebuild + CLAUDE.md reorganization (2026-05-28)

## Context

Project is finished. The primary deliverables are:
- Heterogeneous dual-robot exploration (Go2W + Go2) on a real SLAM-scanned ops2 building
- Centralized vs decentralised CFPA2 coordination under inter-robot comms dropout (the benchmark)
- Full autonomy stack running native on the Orin NX (Point-LIO + trav CNN + CUDA-MPPI + CFPA2 C++)
- Desktop standalone single-robot exploration of the ops2 scene (validated bidirectional ±35 m)

Current state is stale:
- **README.md** (324 lines): references removed features (door task, VLM demo, Swarm-LIO2 as main path, nav_test_demo3 as production), missing the actual final deliverables (dropout benchmark, HIL bench, dual-robot ops2 mixed).
- **CLAUDE.md** (1073 lines): chronological dev journal with "Active state (2026-05-xx)" entries stacked by date. Correct facts are buried inside narrative. Not usable as a reference doc.

---

## Approach: Option A

- Append all "Active state" sections from current CLAUDE.md to CLAUDE1.md (complete development archive).
- Rewrite CLAUDE.md as a concise technical reference (~350–400 lines) organized by component.
- Rebuild README.md from scratch as the external-facing document (~220–260 lines).

---

## README.md Design

**Goal:** External reader can understand what the system is, what was achieved, and how to run the three main scenarios in under 5 minutes.

### Structure

```
1. Title + one-paragraph description
2. Key Results (3-column table)
3. System Architecture (ASCII diagram)
4. Prerequisites & Build
5. Quick Start
   a) Desktop standalone sim (single-robot ops2)
   b) Desktop standalone sim (dual-robot ops2 + dropout benchmark)
   c) Hardware-in-the-Loop (Orin NX)
6. Onboard deployment (brief, → jetson_ws/README.md)
7. Repo layout (trimmed to live packages only)
8. Deeper documentation index
9. Environment table
```

### Key Results table (to include)

| System | Metric | Value |
|---|---|---|
| CFPA2 C++ port | Tick p95 (Orin Nano) | 1.1 ms (was 1376 ms Python, ~1250×) |
| CUDA-MPPI | Kernel chain (Orin Nano, measured) | ~1.15 ms GPU vs ~12 ms CPU (~10×) |
| Full stack (Orin NX) | CPU / GPU / Temp at runtime | <33% / <19% / 56°C |
| Single-robot ops2 | Corridor coverage | Bidirectional ±35 m validated |
| Dual-robot ops2 | Coordination modes × dropout | Centralised vs decentralised, 0/30/50/80% |

### Architecture diagram (final stack)

```
MuJoCo ops2 (500 Hz)  OR  Mid-360 LiDAR (real/HIL)
  └─ mujoco_sensor_bridge / livox_ros_driver2
        │
        ├─ Fast-LIO2 (desktop sim)  ──┐
        ├─ Point-LIO (Orin NX)      ──┤─→  /odom/nav  +  TF map→base_link
        └─ fast_lio_tf_adapter          └─ (via fast_lio_tf_adapter)
        │
        ├─ elevation_mapping_cupy  →  /elevation_map_raw
        │   └─ grid_map_filters     →  /elevation_map_filtered
        │       └─ grid_map_to_occ  →  /<ns>/traversability_grid
        │
        ├─ CFPA2 C++ node  →  /<ns>/way_point_coord
        │   └─ cfpa2_to_nav2_bridge →  /<ns>/goal_pose
        │
        └─ Nav2 (SmacPlannerLattice + CUDA-MPPIController)
             └─ stuck_watchdog  (outer-loop stall recovery)
```

### Quick Start scenarios

**Single-robot ops2 (Go2 walking — not Go2W):**
```bash
./scripts/launch/nav_test_slam_ops2_v4_go2.sh
```
> **Note to include in README:** The ops2 scene uses the full SLAM-reconstructed
> mesh (80×32 m, 982k vertices). MuJoCo collides `<geom type="mesh">` via the
> convex hull of the entire mesh — at this scale that hull engulfs the robot's
> standing volume, making the Go2W's wider/heavier footprint collision-unstable.
> Single-robot ops2 therefore uses **Go2** (Menagerie walking quadruped) only.
> The dual-robot mixed launch avoids this by replacing the mesh collision with
> 44 hand-traced wall boxes (`slam_ops2_v4_mixed_handwalls.xml`), which is why
> Go2W + Go2 works there but not in the single-robot visual-mesh scene.

**Dual-robot ops2 (heterogeneous Go2W + Go2, hand-traced collision walls):**
```bash
./scripts/launch/nav_test_slam_ops2_v4_mixed.sh
```

**Dropout benchmark (centralised vs decentralised, multi-trial):**
```bash
# Default: MODES="centralised decentralised", DROPOUTS="0.0 0.3 0.5 0.8", 10 trials × 600s
./scripts/bench/benchmark_decentralisation_dropout.sh

# Summarize + plot
python3 scripts/bench/summarize_dropout_benchmark.py /tmp/dropout_bench/<ts>/
```

**HIL bench (Orin NX runs full native ROS1 stack, laptop runs sim):**
```bash
./scripts/launch/hil_orin_nx.sh up            # start both sides
./scripts/launch/hil_orin_nx.sh up lidar_range=2.5 max_vel=0.4
./scripts/launch/hil_orin_nx.sh stop
# Full setup: HOW_TO_RUN_HIL.md
```

### What to remove from current README

- `door_demo_mujoco.sh` section (removed feature)
- VLM Integration section (Phase 1 legacy)
- `vlm_demo_mujoco.sh` entry
- Swarm-LIO2 section and `nav_test_swarm_lio2_se2.sh`
- Navigation backends table with all legacy entries (astar, default, reactive)
- Full SLAM options table (keep 2 rows: Fast-LIO2 and Point-LIO)
- Golden Rules section (→ belongs in CLAUDE.md only)
- Debug shell commands block (→ CLAUDE.md)
- `setup.sh` reference (doesn't appear to exist now)

---

## CLAUDE.md Design

**Goal:** Maintainer reference. Someone joining the project should be able to find the answer to "what is the correct current config for X" in under 2 minutes without reading chronological narrative.

### Structure (~350–400 lines)

```
# CLAUDE.md — Collab_QRC Technical Reference
(one-line: project is complete; development history archived in CLAUDE1.md)

## 1. System Overview (~25 lines)
   Final tech stack in one place. Key numbers. Real-time verification results.
   Nav2 / CUDA-MPPI + CFPA2 C++ + Point-LIO + ETH trav.

## 2. CUDA-MPPI PathFollow Fix (~25 lines)
   The PathFollow carrot bug and fix (Rule 25 expanded).
   Speed cap, wheel-engage gate, key yaml paths.

## 3. SLAM (~20 lines)
   Point-LIO (Orin NX production), Fast-LIO2 (sim/desktop),
   fast_lio_tf_adapter role, SIM_GT_ODOM flag.

## 4. Traversability Pipeline (~30 lines)
   elevation_mapping_cupy → grid_map_filters → OccupancyGrid.
   Pure-cupy CNN (Blackwell), visited corridor, wall_cost dilation fix.
   Key yaml paths and rebuild note.

## 5. CFPA2 Frontier Allocation (~40 lines)
   C++ port: hexagonal isolation (core/ros2/ros1).
   cfpa2_executable_suffix:=_cpp.
   Centralised vs decentralised, coordination_mode arg.
   Key params, ig_mode: local (floodfill stub note).

## 6. Ops2 Scene + Exploration Config (~25 lines)
   Scene assets (slam_ops2_v4_mixed_handwalls.xml), spawn points.
   Key fixes: blind disk, visited corridor, extent-seek, inflation 0.16.
   cfpa2_single_robot_ops2.yaml overlay.
   **Robot choice note:** single-robot ops2 uses Go2 (not Go2W). The full SLAM
   mesh collision hull (80×32 m whole-scene convex hull) engulfs the Go2W's
   standing volume at this scale → collision-unstable. Dual-robot mixed works
   with Go2W because it swaps to 44 hand-traced wall boxes instead.

## 7. Dropout Benchmark (~20 lines)
   benchmark_decentralisation_dropout.sh tunables.
   Output layout, summarize + plot tools.
   odom_divergence_monitor, comms_dropout_relay.

## 8. HIL Architecture (~25 lines)
   Laptop sim + Orin NX compute split.
   hil_orin_nx.sh usage. UDP ports (9001–9005).
   Lifecycle gating (platform_ready), NX resource use.
   → HOW_TO_RUN_HIL.md for full setup.

## 9. Onboard Deployment (Orin NX) (~20 lines)
   catkin ws path, onboard_autonomy_noetic.sh usage.
   Key gotchas (nvcc brace-init, sm_89 gating, trav CNN deps).
   → jetson_ws/README.md.

## 10. Build (~15 lines)
    colcon command, COLCON_IGNORE list, YAML-is-live note.

## 11. Scripts Layout (~10 lines)
    launch / bench / runtime / real / debug / ops

## 12. Golden Rules (~55 lines)
    All 25 rules kept (condensed where redundant with sections above).

## 13. Skill-API docs index table (~15 lines)
    Preserve all pointers to docs/claude/*.md.
```

### What to do with the "Active state" sections

All "Active state (2026-05-xx)" entries (lines ~1–800 of current CLAUDE.md) append to CLAUDE1.md under a `## Development History (2026-05-xx)` section header. These are:
- Active state (2026-05-25): CUDA-MPPI PathFollow carrot fix
- Active state (2026-05-20 night): Full autonomy on Orin NX
- Active state (2026-05-20 cont.): Desktop ops2 standalone exploration
- Active state (2026-05-20): Orin Nano HIL day
- Active state (2026-05-19 night): Native ROS 2 CUDA-MPPI plugin
- Active state (2026-05-19 evening): Nav2 SmacLattice + MPPI ROS1 port
- Active state (2026-05-19): CFPA2 pure C++ port
- Active state (2026-05-18 late night): Jetson Orin Nano HIL bag-replay
- Active state (2026-05-18 evening): bridge-as-obstacle root-cause hunt
- Active state (2026-05-18): ops2 trav-CNN fine-tune
- Active state (2026-05-16): real-world walk → MuJoCo collidable scene
- Active state (2026-05-15): ETH elevation mapping live
- Active state (2026-05-14): 3D frontier exploration unstuck
- Active state (2026-05-13): Point-LIO + gbplanner3 onboard
- Active state (2026-05-11): Noetic FAST-LIO2 onboard

---

## Spec Self-Review

- **Placeholders:** None. All filenames and numbers drawn from actual repo state.
- **Consistency:** README Quick Start scripts exist and match current launch files. CLAUDE.md sections map to real components. Golden Rules section (12) is complete — all 25 rules preserved.
- **Scope:** Focused. No new features, no new code. Pure documentation reorganization.
- **Ambiguity:** CLAUDE1.md append order (newest first vs oldest first): spec does not specify — use newest first (most recent sessions appear first in the archive, matching existing CLAUDE1.md style of chronological-descending appended content).
- **Removal decisions:** Legacy entries (door task ref, VLM section, Swarm-LIO2 main path, `setup.sh`) confirmed removed per CLAUDE.md 2026-05 cleanup notes. `nav_test_demo3` series still exists on disk but is not a primary deliverable — listed under "other sim launches" if at all.
