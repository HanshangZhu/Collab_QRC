# CLAUDE.md — Collab_QRC Index

Multi-robot autonomy with Unitree Go2W wheeled-legged quadrupeds + Go2 walking quadrupeds on ROS 2 Humble + MuJoCo (primary) / Gazebo Classic (legacy) / real-robot deployment. Active focus: **Nav2 SE2 holonomic stack tuning** for both single-robot exploration and heterogeneous dual-robot (Go2W + Go2) coordination, with CFPA2 frontier allocation.

Door task (Phase 2 dual-robot VLM coordination) and the legacy A*/default Python nav backends were removed in the 2026-05 cleanup; see [CLAUDE1.md](CLAUDE1.md) for Phase 1 VLM exploration history, Phase 2 FSM archive, archived 2026-04 operational notes, and the deletion log.

## Active state (2026-05-20 night) — FULL autonomy stack running NATIVE on the real Go2 Orin NX (ROS 1 Noetic, no bridge) — verified real-time end-to-end

**The milestone:** the entire autonomy stack now runs **onboard the real Go2's Jetson Orin NX 16 GB** in native ROS 1 Noetic — SLAM (Point-LIO) + traversability (elevation_mapping_cupy + CNN) + Nav2-port nav (SmacLattice global + **CUDA-MPPI** local via `move_base`) + CFPA2 frontier exploration (C++). No ros1_bridge in the data path; the laptop is only NAT internet + SSH (+ RViz2 during HIL). Consolidated catkin workspace at `/home/unitree/autonomous_exploration_zhu/`, mirrored to the repo at [`jetson_ws/`](jetson_ws/README.md).

### Verified real-time on the bench (robot standing, joints locked — pure data-flow validation)

| Topic | Rate | Source |
|---|---|---|
| `/livox/imu` | 200 Hz | Mid-360 |
| `/robot/Odometry` | 10 Hz | Point-LIO (no z-drift) |
| `/robot/odom/nav` | 10 Hz | topic_tools relay (Odometry→odom/nav) |
| `/robot/cloud_registered_body` | 10 Hz | Point-LIO |
| `/robot/traversability_grid` | 5 Hz | elevation_mapping_cupy + filter |
| `/robot/move_base/global_costmap/costmap` | 2 Hz | move_base (trav grid static layer) |
| `/robot/cmd_vel` | 20 Hz | move_base (SmacLattice + CUDA-MPPI) |
| `/robot/way_point_coord` | 2 Hz | CFPA2 (tick p95 0.3 ms) |

All 12 nodes alive, zero DEAD, TF `map→camera_init→body→base_link` connected. **Orin NX resource use with the full stack live: RAM 6.6/15.4 GB, CPU < 33 %, GPU < 19 %, 56 °C, 6.6 W — enormous headroom** (CUDA-MPPI is nearly free on the Orin Ampere GPU; C++ CFPA2 + Point-LIO iVox leave the CPU mostly idle).

### NX hardware / OS
- `unitree@192.168.123.18` (pwd in gitignored [`scripts/real/.orin_nx_cheatsheet.md`](scripts/real/.orin_nx_cheatsheet.md)). Ubuntu 20.04 aarch64, JetPack 5.1.1 / L4T R35.3.1 / CUDA 11.4 / Python 3.8, ROS Noetic.
- NX has **no internet** on the Go2 net; share the laptop's via iptables NAT (recipe in cheatsheet). Go2 Type-C is a USB data port, **not** video-out — drive the NX headless over SSH.
- USB-Ethernet dongle (ASIX AX88179) flaps intermittently → all NX ops use detached `setsid` scripts + short-connection retries so an SSH drop never kills onboard work.

### What it took (the hard parts, full detail in [`jetson_ws/README.md`](jetson_ws/README.md) + the cheatsheet)
- **CFPA2 Noetic port**: 6 `ros1/` adapter headers (clock/logger/conversions/goal_publisher/visualizer/param_facade) mirroring `ros2/`, behind `#ifdef CFPA2_ROS1`. The ROS 2 Humble build is byte-for-byte unchanged (verified — colcon builds cfpa2 clean). `package_ros1.xml`/`CMakeLists_ros1.txt` swap-in; `cfpa2_peer_coordination_msgs` `builtin_interfaces/Time`→`time`; ament-python `cfpa2_peer_coordination` `CATKIN_IGNORE`'d.
- **Build-dep marathon**: grid_map suite, OMPL, move_base/nav_core/costmap_2d, **xtensor 0.24.7 + xtl 0.7.5 vendored into `/usr/local`** (not in focal apt), nlohmann-json, CUDA on PATH (nvcc 11.4 not on default PATH).
- **nvcc 11.4 parse bug**: `rclcpp::Logger logger_{rclcpp::get_logger("…")}` (brace-init) is mis-parsed by nvcc's host preprocessor after the full ROS header set → collapses to `logger_rclcpp`. Isolated to brace-init (console.h/node_handle.h alone are fine; the combination + brace-init breaks). Fix: copy-init `= rclcpp::get_logger(…)` in 6 nav_algo_core headers (math-neutral; g++/nvcc/ROS2 all accept).
- **sm_89 unsupported by CUDA 11.4**: `nav_algo_mppi_cuda` gencode is now CUDA-version-gated — sm_87 always, sm_89 only ≥11.8, sm_120 only ≥12.8.
- **trav CNN runtime chain**: cupy-cuda11x, **torch (NVIDIA jp512 cp38 wheel)**, simple_parsing/ruamel/shapely/sklearn, **scipy==1.10.1** (system 1.3.3's `np.typeDict` is gone in numpy 1.24 → cupy import crash), ros_numpy patches: `np.float`→`float` AND **sort PointFields by offset** (Point-LIO's `cloud_registered_body` lists fields out of offset order → `fields_to_dtype` itemsize≠point_step → "buffer size must be a multiple of element size"). emc API: `_map.input`→`_map.input_pointcloud`; config needs `channels: []`.
- **Wiring**: CFPA2 reads pose from `/<ns>/odom/nav` (hardcoded) → topic_tools relay from Point-LIO's `/Odometry`. CFPA2 ops2 overlay's `planning_map_topic_suffix=/global_costmap/costmap` is Nav2 naming; ROS 1 move_base nests costmaps under `/move_base/…`, so the onboard launcher overrides it to `/traversability_grid` (CFPA2 plans on the trav grid directly). CFPA2 yamls (`/**:ros__parameters:`) are flattened for rosparam via [`scripts/real/generate_cfpa2_ros1_yaml.py`](scripts/real/generate_cfpa2_ros1_yaml.py).
- **exec-bit gotcha**: rsync drops `+x` on `trav_pipeline_ros1/scripts/*.py` → roslaunch "Cannot locate node". `chmod +x` after every deploy.

### Files / entry points
- [`jetson_ws/`](jetson_ws/) — deployment snapshot of the NX catkin ws (flat `src/`). [`jetson_ws/README.md`](jetson_ws/README.md) = packages, data flow, build, run, gotchas.
- [`scripts/real/onboard_autonomy_noetic.sh`](scripts/real/onboard_autonomy_noetic.sh) — onboard full-stack orchestrator (roscore→livox→Point-LIO→TFs→trav→move_base→CFPA2→bridge→odom relay). `explore=false` for nav-only, `slam=fastlio` to swap SLAM, `stop` to tear down.
- [`scripts/real/generate_cfpa2_ros1_yaml.py`](scripts/real/generate_cfpa2_ros1_yaml.py) — flatten CFPA2 ROS 2 yamls → ROS 1 rosparam.
- `jetson_ws/src/trav_pipeline_ros1/scripts/cfpa2_to_movebase_bridge.py` — CFPA2 `way_point_coord` (PointStamped) → `move_base_simple/goal` (PoseStamped) with goal-change suppression + move_base `/status` ABORTED→nav_status fast-blacklist.
- **Repo cleanup**: removed redundant `src/vendor/{point_lio_ros1,trav_pipeline_ros1,elevation_mapping_cupy_ros1,fast_lio_ros1}` (now canonical in `jetson_ws/`). **KEPT `src/vendor/nav_algo_ros1`** — the ROS 2 `nav2_mppi_controller_cuda_plugin` compiles its `.cu` sources directly from there. `deploy_noetic_to_jetson.sh` marked SUPERSEDED.

### 2026-05-21 HIL / real-run operator updates

- **`lidar_range` now means radial point range, not height.** `scripts/real/real_run_monitor.sh` forwards `lidar_range=N` to `scripts/real/onboard_autonomy_noetic.sh`, which patches the active Point-LIO / FAST-LIO `mid360.yaml` `mapping.det_range=N`. This is a spherical/polar point radius cutoff in the SLAM preprocess/mapping layer. It does **not** change the Livox driver raw UDP output and it does **not** change the traversability ceiling.
- **Height filtering is separate.** Use `trav_height_range=N` for the elevation/traversability gate. The onboard launcher patches `trav_pipeline_ros1/config/elevation_mapping_go2w.yaml` `max_height_range` and `ramped_height_range_b`.
- **Laptop RViz2 is now bidirectional.** `scripts/launch/nx_viz_laptop.sh` runs ROS 2 UDP RX for NX viz topics and ROS 2 UDP TX for RViz `/goal_pose` → NX `/robot/move_base_simple/goal`. Manual mode is `explore=false`; the robot should still receive 2D Goal Pose via the relay.
- **RViz2 planned path relay.** ROS 1 NX `hil_relay_tx_node` forwards `/<ns>/move_base/SmacLatticePlannerROS/plan` (`nav_msgs/Path`) over UDP `plan_port=9008`; ROS 2 laptop `hil_relay_rx_node` republishes `/robot/planned_path`. `hil_nx.rviz` displays this as `Plan Path`.
- **RViz2 visual style.** `src/go2w/go2_gazebo_sim/rviz/hil_nx.rviz` is black/white/gray for report-friendly HIL viewing: black background, gray traversability cloud/grid, white odometry/path/frontier marker, gray manual goal marker.
- **CycloneDDS interface helper.** Real-run laptop execution must bind DDS to the currently active `192.168.123.x` NIC, not stale `enxc8a36240a4c7`.
  - [`scripts/real/laptop_ethernet_setup.sh`](scripts/real/laptop_ethernet_setup.sh): owns `GO2W_ETH_IFACE` auto-detect, stale `CYCLONEDDS_URI` cleanup, `connect_ethernet.sh` sourcing, and DDS interface sanity check.
  - [`scripts/real/real_run_monitor.sh`](scripts/real/real_run_monitor.sh): orchestration only; sources `LAPTOP_ETHERNET_SETUP_SCRIPT` and calls `setup_real_monitor_laptop_ethernet`.
  - [`scripts/test/test_real_monitor_laptop_ethernet_decoupling.sh`](scripts/test/test_real_monitor_laptop_ethernet_decoupling.sh): regression guard preventing Ethernet setup logic from drifting back into the monitor.
- **Real execution is not CHAMP.** Sim CHAMP drives a simulated joint-level/CHAMP controller. The real-run path is `move_base` MPPI → `/robot/cmd_vel` → laptop ROS 2 `cmd_vel_to_sport_bridge.py` → Unitree Sport API `/api/sport/request` with obstacle avoidance off. Low-speed real Go2 walking can look like torso yaw with weak translation if `linear.x` is below gait response/deadband or if the Sport API subscriber/control mode is not active. Debug this by comparing `/robot/cmd_vel`, `/api/sport/request`, and `/tmp/laptop_cmd_vel_to_sport.log` before retuning the planner.

### Open / next
- **HIL viz**: C++ ROS 1→ROS 2 bridge for viz topics (map/costmap/cloud/path/tf/cmd_vel) + RViz2 on the laptop, so the operator sees everything while compute stays on the NX. Then a full "sim" dry-run on the ops2-v4 scene (`nav_test_slam_ops2_v4_go2`) to measure load/trajectory/dataset.
- **Real walk**: unlock joints, robot on ground, clear area, e-stop ready → same `onboard_autonomy_noetic.sh explore=true`. cmd_vel already streams; unlocking lets the robot drive CFPA2 frontiers.
- **`/robot/elevation_map_raw`** shows no `rostopic hz` (the GridMap may publish under a different name / lower rate) — the downstream `/traversability_grid` flows at 5 Hz so nav is unaffected, but worth confirming the raw layer name during the HIL run.

## Active state (2026-05-20 cont.) — Desktop ops2 standalone exploration UNSTUCK: 8-bug cascade fixed, robot now autonomously explores the corridor

Goal: make the Go2 autonomously explore the ops2 building corridor end-to-end on the **desktop standalone** stack (`scripts/launch/nav_test_slam_ops2_v4_go2.sh`, all-in-one — MuJoCo + Fast-LIO + elevation/trav + Nav2 + CFPA2, no Jetson). The robot was pinned at spawn (0,0). Fixed a cascade of independent bugs, each masking the next; the robot now navigates frontier-by-frontier down the corridor. Built a reusable auto-diagnosis + cleanup toolchain along the way.

### The bug cascade (in the order they surfaced — each blocked discovery of the next)

1. **CFPA2 reachability config divergence.** Desktop used the demo_ramp overlay (`allow_unknown=true` → reachability BFS leaks through unknown behind thin hand-walls); HIL Jetson used base yaml (`max_goal_distance 2.5 m` → stalls at no_frontiers on a 70 m corridor). Fix: one unified [`cfpa2_single_robot_ops2.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot_ops2.yaml) overlay used by BOTH desktop (`nav_test_3d_explore` `cfpa2_config_overlay` arg) and the HIL Jetson launch.
2. **Python CFPA2 node is broken** (`ImportError: attempted relative import`). Desktop defaulted to it. Fix: ops2 launcher passes `cfpa2_executable_suffix:=_cpp` (the production C++ binary).
3. **200×200 m / 2000² costmap** → SmacHybrid `allow_unknown` search wandered 4 M cells → ~17 s planner timeouts. Fix: shrank the fixed trav grid to 100×100 m / 1000² (covers ±50 m).
4. **Fast-LIO z-drift in standalone sim.** Not lifecycle-gated on the desktop → inits gravity during stand-up → `/odom/nav` z drifts 0.28→1.3 m → garbage for CFPA2/Nav2. Fix: `SIM_GT_ODOM=1` (default in the ops2 launcher) feeds `/odom/nav` from MuJoCo ground truth; the trav grid stays REAL perception (body-frame cloud + GT TF). Real robot/HIL keep gated Fast-LIO.
5. **Trav-grid SELF-PAINTING** (THE planner-abort cause). The Go2's own body/leg LiDAR returns (beyond `min_valid_distance 0.30`) painted a LETHAL blob on the robot's footprint → SmacHybrid start-in-collision → `PLAN_OK=0` always, even for a 1.5 m goal. The per-frame footprint seed (`stamp_free_disk`, `seed_max_clear_cost 50`) refused to clear cost-100 self-paint. Fix: two-tier seed in [`grid_map_to_occupancy_grid.py`](src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py) — inner `robot_core_clear_radius_m 0.40` clears the footprint UNCONDITIONALLY, outer seed stays conditional; plus `min_valid_distance 0.30→0.40`. **Required a rebuild** — ament_cmake_python installs the executable as a COPY, not a symlink, so source edits don't take effect until `colcon build --packages-select trav_cost_filters`.
6. **Cross-host Jetson stale stack (the "trav-grid flashing").** A leftover `orin_nano_hil_jetson.launch.py` on the Jetson (same `/robot` namespace + DDS domain) was publishing `/robot/traversability_grid`, `/robot/tf`, goal_pose, cmd_vel → "Publisher count: 2", stale costmap, RViz flashing, goal churn. Desktop-only cleanup can NEVER fix this. Fix: hardened [`scripts/debug/kill_sim.sh`](scripts/debug/kill_sim.sh) (kills desktop + SSH-kills Jetson + verifies) and the ops2 launcher best-effort SSH-kills the Jetson stack at preflight (`SKIP_JETSON_PREFLIGHT=1` to opt out).
7. **Exploration BT + install gap.** The default no-spin BT clears the GLOBAL costmap on every plan failure → wipes the accumulated exploration map → CFPA2 candidate list empties → stale-goal/clear loop. Fix: [`navigate_to_pose_explore.xml`](src/go2w/go2w_config/config/nav/behavior_trees/navigate_to_pose_explore.xml) (clears LOCAL only) used in explore mode. New BT files also need installing (symlink into `install/go2w_config/.../behavior_trees/`) or bt_navigator fails to load → "unknown goal response" flood.
8. **Mid-360 geometric blind disk → robot bubble disconnected.** The trav grid is UNKNOWN at 1-4 m around the robot (V-FOV starts ~-7° → ground first visible ~3 m) but FREE at 5 m+. The 0.65 m seed bubble was isolated from the sensed-free ring by the unknown blind ring → BFS (`allow_unknown=false`) couldn't cross → 1.2 m² reachable, no escape. Fix: restored the **3.0 m forced-free disk** (`robot_seed_radius_m 0.65→3.0`, conditional so real walls stay) → 1.2 m²→132 m² reachable.
9. **THE FINAL BUG — `cfpa2_ig_mode: floodfill` is a STUB.** The C++ port's single-call `frontier_information_gain_floodfill()` returns **0.0** (only the BATCH path implements floodfill). `cfpa2_single_utility` (used in the candidate filter) calls the single-call path → `info_gain 0 < 3.0` → utility -1e18 → EVERY frontier rejected → util list empty → CFPA2 publishes NO goal → robot frozen. My own ops2 overlay had set `ig_mode: floodfill`. Fix: `cfpa2_ig_mode: "local"` (the real box-count IG, implemented in the single-call path). **Result: robot immediately started navigating** — spawn (0,0) → (3.6, -4.6) in 75 s, exploring down the corridor.

### Debug/automation toolchain shipped (the user asked for auto-diagnosis)

- [`scripts/debug/stuck_diagnoser.py`](scripts/debug/stuck_diagnoser.py) — auto-classifies WHY the robot stopped: NO_GOAL / NO_PLAN / CONTROLLER_IDLE / WALL_HIT / TRAV_CORRUPT. Triggers off the stuck_watchdog's `/<ns>/recovery_event` OR self-detected stillness; probes costmap+trav+plan+cmd_vel; writes a verdict block to stdout + JSONL log. Auto-wired into `nav_test_mujoco_fastlio.launch.py` explore mode (disable via `STUCK_DIAGNOSER=0`).
- [`scripts/debug/trajectory_monitor.py`](scripts/debug/trajectory_monitor.py) — tracks min/max x (the ±35 validation oracle), path length, correlates CFPA2 status + verdicts; JSON summary on exit.
- [`scripts/debug/explore_autorun.sh`](scripts/debug/explore_autorun.sh) — one-command run: launch sim + monitor + diagnoser, heartbeat, final report.
- [`scripts/debug/kill_sim.sh`](scripts/debug/kill_sim.sh) — hardened two-host teardown (desktop + Jetson) + verify; reliable PID/comm/arg kills (pkill -f alternation silently misses; nav2 comm names truncate >15 chars; launch roots trap SIGTERM).

### Status of the ±35 full-corridor goal (honest)

**Achieved + validated:** the robot autonomously explores and reliably reaches the **+x end (x_max ≈ 32, within 10 % of +35)** across runs 15/17/19/21/23, beelining down the corridor at ~0.1 m/s. With the fallback-snap (below) it explores ROBUSTLY — path 80 m+ with **no freeze**.

**Not yet (the −x branch):** run23 (15 min) confirmed the failure mode precisely. The ops2 building is **V-shaped** (handwall extent x=[−37.3, +41.7], spawn at the middle (0,0)): the **+x branch** goes down-right to x≈+41 (fully explored), the **−x branch** is a *winding diagonal maze* going down-LEFT to x≈−37. Probes on the live run:
- Nav2 plans `OK 330` back to **spawn (0,0)** but `EMPTY` to every more-negative point → the −x branch is unexplored UNKNOWN, not blocked; the robot just never pushed into it.
- The reachable BFS component (global costmap) spans x=[−8.5, +39.7] (34.9k cells) — it DOES reach spawn, but all 73 frontiers are `unreachable` (across unknown/inflation-narrowed gaps), so the only thing that can drive −x is the fallback-snap.
- The fallback-snap WAS emitting −x goals (`-8.4,-8.3`, `-3.8,-3.0`) but **thrashing** every tick (GOAL_SENT jumped +36→+4→−8→+25…) because its utility included velocity-dependent momentum (flips while circling) and the TSP head picked the *nearest* snapped goal (changes every tick) → Nav2 never committed → robot circled +x hitting walls (107 WALL_HIT). x_min frozen at −2.49 since t=60 s.

### Two MORE fixes this session (12–13) targeting the −x branch — applied, validating in run24

12. **Persistent visited-corridor stamping** ([`grid_map_to_occupancy_grid.py`](src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py)). A persistent boolean `_visited_mask` accumulates the **swept capsule** between consecutive robot poses (`visited_corridor_radius_m 0.45`, interpolated so fast motion can't leave a gap) and forces those cells FREE on every publish — **ground truth**, the robot physically occupied them, so it can never erase a real wall; it only repairs the Mid-360 blind-zone holes the rolling elevation map leaves behind the robot. Keeps the reachable component connected back through spawn permanently (the recommended fix from the prior entry, now implemented). Default ON (`visited_corridor_enabled`).
13. **Fallback-snap made deterministic + stable** ([`cfpa2_coordinator.cpp`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_coordinator.cpp)). The fallback utility is now **IG-only** (dropped distance/switch/**momentum** — momentum was the velocity-dependent thrash source), and after the sort the list is **truncated to the single max-IG snapped goal** so the TSP head can't pick a jumpy nearer alternate and the goal-policy switch-hysteresis holds it. The deterministic winner is the largest-unexplored-region frontier (the −x corridor), whose snapped reachable cell LEADS the robot into it; once −x cells get sensed they become reachable → the normal path (with momentum now HELPING maintain −x heading) takes over. Both packages rebuilt (trav_cost_filters copy-install, cfpa2 C++).

### Two more CFPA2 fixes shipped this session (beyond the 9 above)

10. **`allow_unknown` was a NO-OP** — `distance_transform_range`'s `is_free` had a blanket `v >= 0` that blocked unknown (-1) regardless of the `unknown_val` param, so `allow_unknown=true` never flooded unknown (dist_map stayed = known-free only). Fixed `is_free` to `v != unknown_val && v < occ_threshold`. (Settled on `allow_unknown=false` anyway — true makes Nav2 plan optimistically through unknown that has walls → oscillation.)
11. **Fallback snap-to-reachable** in CFPA2 Pass-1 ([`cfpa2_coordinator.cpp`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_coordinator.cpp)) — when NO real reachable frontier survives (robot exhausted its connected component; remaining frontiers are across an unentered corridor that reads unreachable because the Mid-360 blind zone fragments the trail), retarget the nearest reachable cell keeping the frontier's IG. Un-sticks the +x-corner FREEZE (run19/21 froze at path ~56 m; run23 sails past to 80 m+) WITHOUT suppressing the normal beeline (it's a fallback, not a competitor — the first version that competed made the robot dawdle near spawn).

### Bidirectional ±x (runs 24–30) — extent-seek + inflation re-tune

Fixes 12–13 (persistent corridor + stable IG-only fallback) did NOT alone reach −x: runs 24/25 showed the robot greedily grinds +x pockets / wedges in the +x corner (high-IG perimeter frontiers fool an IG-override). The bidirectional breakthrough came from an **extent-seek strategy** (turn around once one ±x extreme is physically reached) made **decisive** (directional guard + always-commit −x goal + bypass goal-policy), then an **inflation re-tune** (`inflation_radius 0.28 → 0.16`) so the 1.0 m winding −x corridor keeps a cost-0 lane wider than the robot's 0.64 m turning envelope.

- **run29 = BREAKTHROUGH:** first real −x traversal (x_min −8.4) after the decisive extent-seek; verified Nav2 *can* path −x (`ComputePathToPose` OK 364) so the only blocker was goal thrash.
- **run29 STALL → run30:** robot wedged at −8.4 because Nav2 inflation crushed the 1.0 m corridor to a 0.44 m cost-0 lane < 0.64 m turning envelope. `inflation_radius 0.16` (cost-0 lane 0.68 m) + `consider_footprint=true` (exact guard) is the fix. **run30 validating to full −31.5.**
- **stuck_diagnoser gained `REAL_COLLISION`** (subscribes `/mujoco/contacts`; body↔wall = real crash, foot↔wall = minor) — distinguishes "wedged" from "actually hit a wall".

**Full deployment-oriented writeup (run-by-run + final config + real-robot checklist):** [docs/claude/ops2_bidirectional_exploration_journey.md](docs/claude/ops2_bidirectional_exploration_journey.md).

### Open / next

- **NOT YET VALIDATED end-to-end:** bidirectional ±x is proven to **x_min −8.4** (run29); full −31.5 pending. run30 showed `inflation 0.16` made the **IG-override** (gated on `best_reach_ig`, not extent) fire during the +x beeline → +37↔−16 thrash → stuck at +28. Fix: **IG-override disabled** in the ops2 overlay (extent-seek alone handles −x). **run31 (override-off + inflation 0.16) needs a clean validation run** — it was killed mid-run for a manual-goal isolation test.
- **Manual-goal isolation test (CFPA2 off):** Nav2 returns EMPTY to deep −x (−30,−10) — unexplored space has no path. Proves there is NO manual/waypoint shortcut to unmapped −x; the incremental explore loop (extent-seek + snap) is the only way. *(stuck_watchdog republishes stale goals when killed-CFPA2 + manual-goal — kill it too for clean manual tests.)*
- **Waypoint navigation (`navigate_through_poses`)** — operator-proposed robustness layer (sample 1 m waypoints from the planned path, replan between them; committed incremental progress + per-meter replan opens previously-blocked routes). Complementary to extent-seek, NOT a fix for too-tight corridors. Full analysis + impl sketch in [docs/claude/ops2_bidirectional_exploration_journey.md](docs/claude/ops2_bidirectional_exploration_journey.md). Add AFTER validating run31.
- **Trav grid over-paints free→lethal — ROOT FOUND + FIXED:** live probe of `/robot/elevation_map_filtered` showed **91% of lethal cells are wall_cost-driven**, only 2% slope/step noise. Cause = `wall_cost_dilated` (grid_map_filters.yaml filter10) `maxOfFinites window_size:5` = ±0.20 m at 0.10 m/cell → every wall painted 0.40 m fatter; static_layer copies it, inflation amplifies. **Fix: window_size 5 → 3** (±0.10 m); re-probe wall-driven lethal 91%→70%. The costmap static_layer copies the trav grid verbatim so trav over-paint is amplified, NOT just inherited — fix the trav grid first. Secondary (minor, ~4% open floor, early-sensing noise): add a `variance`-confidence gate (sparse→UNKNOWN not lethal) if it matters on the real robot. Detail in the journey doc.
- **Real-robot re-verify:** re-tune `inflation_radius`/`footprint` against ACTUAL corridor widths (SLAM mesh ≠ real building); do NOT set `SIM_GT_ODOM` (lifecycle-gated Fast-LIO instead); `/mujoco/contacts` collision detection is sim-only.
- **`cfpa2_ig_mode: floodfill` single-call stub** should be implemented (or the param removed) — silently zeroes IG (footgun; ops2 overlay pins `local`).
- New files need install artifacts (trav_cost_filters/cfpa2 C++ need a `colcon build`; new BT/yaml need symlinks) — verify after adding.

## Active state (2026-05-20) — Orin Nano cross-host HIL: lifecycle gating, trav-grid debug, hand-drawn walls, trav-CNN training pipeline

Full day on the Orin Nano HIL bench (desktop runs MuJoCo ops2 sim, Jetson `johnpork233@192.168.55.49` runs the autonomy stack — Fast-LIO + elevation_mapping + Nav2 CUDA-MPPI + CFPA2). Shipped a unified launcher, a 2-tier trav-CNN training pipeline, and fixed a cascade of HIL bugs. NOT yet committed when this entry was written — see "Open / next" for the live nav blocker.

### TL;DR — what shipped

- **Unified HIL launcher** [`scripts/launch/hil_orin_nano.sh`](scripts/launch/hil_orin_nano.sh): single entry for BOTH sides. `up` = preflight-kill both → start desktop sim → wait "platform Ready" → start Jetson autonomy → wait "CUDA backend ENABLED". `stop` / `status` / `monitor` subcommands. Solves the time-jump-back race (desktop sim restart resetting sim_time mid-run cleared all Jetson tf2 buffers) by always starting desktop FIRST and waiting.
- **Persistent Jetson logs**: [`run_jetson_hil.sh`](scripts/real/run_jetson_hil.sh) tees to `~/jetson_ws/logs/jetson_hil_<ts>.log` + `latest.log` symlink, last-30 retention. [`scripts/real/fetch_jetson_logs.sh`](scripts/real/fetch_jetson_logs.sh) pulls them to desktop (all/latest/`tail -f`).
- **Fast-LIO lifecycle gating — THE big fix.** Fast-LIO was doing IMU init DURING the CHAMP stand-up leg motion → wrong gravity vector → **z drifted to +5.8 m** (robot "flying", GT z=0.28m) → every cloud projected to wrong world pos → trav grid was a spiral-fan smear during rotation. Fixed with a 5-piece cross-host lifecycle handshake: (1) [`stand_up_slowly.py`](src/go2w/go2w_spawn/scripts/stand_up_slowly.py) holds open 20 s until the trajectory is physically complete; (2) [`wait_for_ready.py`](src/go2w/go2w_spawn/scripts/wait_for_ready.py) gains a `signal_topic` param → publishes latched `Bool(True)`; (3) a 2nd `wait_for_settle` gate in [`single_go2w_mujoco_cfpa2.launch.py`](src/go2w/go2_gazebo_sim/launch/single_go2w_mujoco_cfpa2.launch.py) chained `OnProcessExit(stand_up_node)` verifies |ω|<0.05 for 5 s then publishes `/robot/platform_ready`; (4) [`run_jetson_hil.sh`](scripts/real/run_jetson_hil.sh) blocks on that latched topic before exec'ing the autonomy launch; (5) `assets.py` exposes `stand_up_node` via `return_handles`. **Result: z drift +5.8 m → −0.14 m.** Fast-LIO IMU init now happens on a settled robot.
- **Hand-drawn walls workflow** (replaces PolyFit overfit). PolyFit auto-RANSAC ([`polyfit_lite.py`](scripts/real/polyfit_lite.py)) took the AABB of all coplanar inliers → 47×10 m slabs that cut across corridors. Two fixes: (a) added DBSCAN inlier clustering + vertical-only + 4 m height-clamp + dedup to polyfit_lite → `slam_ops2_v4_go2_clustered.xml` (68 tight boxes vs 19 oversized); (b) NEW [`scripts/training/draw_walls_2d.py`](scripts/training/draw_walls_2d.py) — interactive top-down hand-trace tool on a full-building heightmap with **z-band sliders** (set z_lo=1.7 m to isolate the elevation-map-FILTERED tall structures like bike racks → trace them as collision geom without polluting the elevation map), magma contrast, polyfit reference overlay, and EDIT mode (drag vertex / click+`d` delete segment / right-click delete vertex). User hand-traced 44 wall segments → `slam_ops2_v4_go2_handwalls.xml` (now the HIL default `POLYFIT_VARIANT=handwalls`). Source artifacts in `bags/meshes/ops2_cuda/handwalls/`.
- **Trav-CNN 2-tier training pipeline** (committed earlier as `40b5e0a` + extended): `mujoco_static_label_map.py` (GT labels via top-down raycast collision), `mujoco_clean_heightmap.py` (clean z(x,y) reference), `synth_noise_corpus.py` (offline pretrain: clean heightmap + injected LiDAR/walking-pitch/pose-drift/Risley-dropout noise), `trav_corpus_collector.py` + `run_with_corpus.sh` (live-sim capture during bench), `merge_trav_corpus.py` (aggregate). All emit train_trav_filter.py-compatible .npz. See [TRAINING.md](TRAINING.md).
- **elevation_mapping_node.py async patch**: `SingleThreadedExecutor` → `MultiThreadedExecutor(4)` + MutuallyExclusive cloud cb-group + Reentrant timer cb-group → elev_raw 3.13→3.79 Hz, Odometry 7.9→10.1 Hz on Orin Nano. Plus `safe_lookup_transform` now WAITs 200 ms for the TF at the cloud's stamp (cross-host lag) instead of silently falling back to the latest pose (which applied a stale-by-100ms pose during rotation = smear).
- **filter_chain trimmed** [`grid_map_filters.yaml`](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml): 30→16 stages (dropped roughness SlidingWindow + ramp_safe chain that's a no-op on flat indoor + redundant clamps). 0.6→1.0 Hz on Orin Nano. **CRITICAL: re-added `wall_cost_clamp_hi`** — without it, a single stray elevation outlier (z=-27m) blew `wall_cost` to 51 → trav_eth went negative → clamped to 0 → 5×5 LETHAL blob per outlier, stuck in fixed_grid. That was the speckled-purple trav grid.
- **trav grid right-sized** [`orin_nano_hil_jetson.launch.py`](scripts/real/orin_nano_hil_jetson.launch.py): `fixed_width/height_cells` 2000→500 (200×200m → 50×50m). The 2000×2000 OccupancyGrid (4 MB/msg) made RViz try a single 2000² GL texture (`Trying to create a map ... using 1 swatches`) → at/over GL_MAX_TEXTURE_SIZE → rendered black; plus DDS reliable+transient_local backpressure cross-host. 500×500 = 250 kB renders fine.
- **pointcloud_adapter deskew window** [`pointcloud_adapter.py`](src/go2w/go2w_perception/scripts/pointcloud_adapter.py): synthetic per-point time span 10 ms → 100 µs. The 10 ms azimuth-based span assumed a Velodyne sweep that doesn't match Mid-360 Risley; under rotation Fast-LIO wrong-deskewed by ±3°. (Helped, but the dominant rotation smear was the z-drift above.)
- **build_float_heightmap.py + polyfit_lite.py**: `sys.modules.setdefault("open3d.ml", ...)` stub so open3d core imports without its eager ml→sklearn→numpy-1.x `_CopyMode` crash.
- **RViz**: restored `nav_test.rviz` to the ops4-matching original (Color Scheme `map`, 25 displays) after an earlier costmap-scheme experiment; HIL desktop now loads it via the new `rviz_config` launch arg. `path_relay.py` was never in the deploy list → synced + added → `/robot/plan` → `/robot/planned_path` (RViz) now works.

### Open / next — live nav blocker (NOT yet fixed)

Robot doesn't explore: **CFPA2 picks frontiers BEYOND a wall** (user's diagnosis, confirmed). CFPA2 reachability BFS reads `/robot/global_costmap/costmap` and leaks through UNKNOWN cells *behind* the thin (0.1 m) handwall → a frontier on the wrong side looks reachable → Nav2's SmacPlannerHybrid (footprint-aware) can't path there → 8 s stuck-recovery → blacklist → picks another cross-wall frontier → oscillation. Verified: failed goals are at cost-0 and point-flood-fill-reachable, but the **footprint check** rejects them — footprint 0.64×0.36 m (circumscribed 0.40 m) > inflation_radius 0.20-0.25 m, so footprint corners hit lethal even at cost-0 centers. Two-part fix for next session: (1) CFPA2 frontier reachability must treat lethal+inflation as a HARD barrier (don't cross wall-gated unknown); (2) align inflation_radius with footprint circumscribed radius OR shrink footprint OR have CFPA2 only pick deeper-free goals. Also requested: waypoint navigation (navigate_through_poses, 1 m spacing) for efficiency + un-sticking. The trav-grid-not-growing symptom is downstream of this (robot can't move → map can't grow).

## Active state (2026-05-19 night) — Native ROS 2 CUDA-MPPI plugin for Nav2 Humble (`nav2_mppi_controller_cuda_plugin`)

The 11-kernel CUDA backend from the Noetic port is now wired **directly into Nav2 Humble's `controller_server`** as a first-class `nav2_core::Controller` plugin — no ros1_bridge, no Docker, no port overhead. Commits `71685f6` (plugin) + `47c2d09` (separator fix + launch script).

### TL;DR

- **`nav2_mppi_controller_cuda_plugin::CudaMPPIController`** is a subclass of `nav2_mppi_controller::MPPIController` that overrides `configure()` to instantiate `nav_algo_mppi_cuda::CudaBackend` and inject it via `optimizer_.setCudaBackend(...)`. When `use_cuda: true` (default in both go2 and go2w yamls), the entire MPPI hot loop (integrate + 8 critics + cost_shape + softmax + weighted_avg) runs on GPU. When `use_cuda: false`, the base class xtensor CPU path runs unchanged.
- **Verified live** via `./scripts/launch/nav_test_cuda_mppi.sh gui:=false rviz:=false`: controller_server logs show all 8 critics loaded + `CUDA backend ENABLED (B=2000 T=56 footprint_n=4)` on configure.
- **Same `.cu` kernels as the Noetic ROS 1 stack.** `cuda_backend.cu` / `critics.cu` / etc. compile against either `nav_algo_core` (ROS 1) or `nav2_mppi_controller` (ROS 2) headers via a single `#ifdef NAV_ALGO_MPPI_CUDA_USE_NAV2` selector defined by each package's CMake.
- **`src/vendor/nav2_mppi_controller_cuda/`** — vendored `nav2_mppi_controller` 1.1.20 renamed + patched: `Optimizer` gets `cuda_backend_` member + `setCudaBackend()` + 10 public accessors; `optimize()` short-circuits to `cuda_backend_->optimize(*this)` when set; `ICudaBackend` interface added in `include/nav2_mppi_controller/cuda_backend.hpp`. Critic class_loader key changed to `nav2_mppi_controller_cuda` so it coexists safely with the apt package.
- **`src/nav2_mppi_controller_cuda_plugin/`** — thin ROS 2 colcon package with CUDA enabled (`LANGUAGES CXX CUDA`). Compiles the 4 `.cu` source files from `nav_algo_ros1/nav_algo_mppi_cuda/src/` directly (no copy). Exports pluginlib descriptor `mppic_cuda.xml` → `libcuda_mppi_controller.so`.
- **pluginlib separator is `::` not `/`** (ROS 2 convention). Both `nav2_go2_full_stack.yaml` and `nav2_go2w_full_stack.yaml` were updated from `nav2_mppi_controller_cuda_plugin/CudaMPPIController` → `nav2_mppi_controller_cuda_plugin::CudaMPPIController`. The `/` form causes a FATAL at configure even though pluginlib *lists* the `::` type in its error message.
- **`nav2_package()` macro must NOT be called** in the plugin's CMakeLists — it adds `-Wdeprecated` to all languages and nvcc rejects it. Use manual `add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>)` instead.
- **`find_package(CUDAToolkit REQUIRED)`** needed so that host-compiled `.cpp` files that `#include <cuda_runtime.h>` can resolve it via `target_include_directories(... PRIVATE ${CUDAToolkit_INCLUDE_DIRS})`.

### Files / paths

- [`src/vendor/nav2_mppi_controller_cuda/`](src/vendor/nav2_mppi_controller_cuda/) — vendored + patched upstream controller package
- [`src/vendor/nav2_mppi_controller_cuda/include/nav2_mppi_controller/cuda_backend.hpp`](src/vendor/nav2_mppi_controller_cuda/include/nav2_mppi_controller/cuda_backend.hpp) — pure-virtual `ICudaBackend` interface
- [`src/vendor/nav2_mppi_controller_cuda/src/optimizer.cpp`](src/vendor/nav2_mppi_controller_cuda/src/optimizer.cpp) — `optimize()` CUDA short-circuit
- [`src/nav2_mppi_controller_cuda_plugin/`](src/nav2_mppi_controller_cuda_plugin/) — ROS 2 colcon plugin package
- [`src/nav2_mppi_controller_cuda_plugin/src/cuda_mppi_controller.cpp`](src/nav2_mppi_controller_cuda_plugin/src/cuda_mppi_controller.cpp) — `configure()` / `cleanup()` overrides
- [`src/nav2_mppi_controller_cuda_plugin/mppic_cuda.xml`](src/nav2_mppi_controller_cuda_plugin/mppic_cuda.xml) — pluginlib descriptor
- [`scripts/launch/nav_test_cuda_mppi.sh`](scripts/launch/nav_test_cuda_mppi.sh) — launch entry (pre-flights .so + ament_index marker, then `exec ros2 launch ... nav_test_mujoco_fastlio.launch.py`)
- `src/go2w/go2w_config/config/nav/nav2_go2{,w}_full_stack.yaml` — `FollowPath.plugin` set to `nav2_mppi_controller_cuda_plugin::CudaMPPIController`, `use_cuda: true`

### Build

```bash
colcon build --symlink-install \
  --packages-select nav2_mppi_controller_cuda nav2_mppi_controller_cuda_plugin
# CUDA arch: sm_87 (Orin Ampere) + sm_89 (RTX 4050) baked in; sm_120 added when CUDA ≥ 12.8
```

### Open / next

- **`footprint_n=4`** in the live run (from nav2_go2_full_stack.yaml default footprint polygon). Check it matches the intended robot footprint and not a degenerate default; `nav2_go2w_full_stack.yaml` explicitly sets a larger polygon.
- **Critic weights hardcoded in `cuda_backend.cu`** (same v1 limitation as the Noetic port) — wire through `CriticManager` accessors.
- **Host-side critic gates** (`within_position_goal_tolerance`, `furthest_reached_path_point`, etc.) not yet implemented in the ROS 2 backend — same v1 scope as Noetic.
- **Orin NX validation** — same `-gencode arch=compute_87,sm_87` flag as the Noetic stack; should deploy identically once the workspace is synced to Jetson.

---

## Active state (2026-05-19 evening) — Nav2 SmacLattice + MPPIController ROS 1 Noetic port + full GPU MPPI pipeline (11 kernels, end-to-end)

Companion to the morning's CFPA2 C++ port (`d0526bf`, see entry below): the Noetic-portability push extended from CFPA2 to the rest of the autonomy stack. Nav2 Humble's SmacPlannerLattice (global) + MPPIController (local) now compile + run on ROS 1 Noetic via `move_base`, mathematically equivalent to the Humble sim. Full 11-kernel CUDA MPPI shipped on top — Optimizer's xtensor hot loop is now dispatched to GPU through a clean `ICudaBackend` interface, verified end-to-end with 200+ live `optimize()` calls. Commits on top of `d0526bf`: `cd4c306` → `068f3ed` → `48de44f` → `35f9a72` (CLAUDE.md) → `d35aa73` → `01c4698` → `8c8107a` → `bc9786c` → `94595ea` → `a00e35c`.

### TL;DR

- **End-to-end pipeline tested in `ros:noetic-ros-core` Docker** (no real Jetson needed for sim): synthetic 100×100 OccupancyGrid + goal at (3, 0) → SmacLatticePlannerROS produces 16-heading lattice plan in ~80 ms → MPPIControllerROS emits `cmd_vel = (vx=0.30, wz=0.0)` at `vx_max` toward goal. Same `cmd_vel` value as Nav2 sim would emit on the same scenario, by construction. With `use_cuda: true` yaml flag the entire MPPI hot loop runs on GPU; verified via probe instrumentation: 200+ `Optimizer::optimize()` dispatches go through `CudaBackend` over 8 s of controller activity at ~25 Hz.
- **13K LOC algorithm body byte-identical to Nav2 Humble upstream.** Every `src/vendor/nav_algo_ros1/nav_algo_core/src/**/*.cpp` diffs against `src/vendor/nav2_humble_src/` showing only 2 lines changed per file: `#include "nav2_*"` → `#include "nav_algo_core/*"`, plus auto-injected `#include "nav_algo_core/compat.hpp"` at top. Math is preserved by construction.
- **11 CUDA kernels covering the entire MPPI hot loop**, every one validated cell-by-cell against a faithful CPU reference (max `|Δ|` < 1e-4 across the board, several at fp32 epsilon ~1e-7 or bit-exact 0):

| kernel | what it replaces | CPU | GPU | speedup | max \|Δ\| |
|---|---|---|---|---|---|
| `integrate` | `integrateStateVelocities` | 2.17 ms | 0.016 ms | **139×** | 3.6e-7 |
| `goal_critic` | `GoalCritic::score` | 0.26 ms | 0.015 ms | 17× | 2.9e-6 |
| `goal_angle_critic` | `GoalAngleCritic::score` | 0.38 ms | 0.015 ms | 25× | 9.5e-7 |
| `prefer_forward_critic` | `PreferForwardCritic::score` | 0.38 ms | 0.014 ms | 27× | 6.0e-8 |
| `constraint_critic` | `ConstraintCritic::score` | 0.73 ms | 0.014 ms | 49× | 7.4e-9 |
| `path_follow_critic` | `PathFollowCritic::score` | 0.027 ms | 0.014 ms | 2× | 3.8e-6 |
| `path_angle_critic` | `PathAngleCritic::score` | 5.96 ms | 0.017 ms | **345×** | 4.8e-7 |
| `path_align_critic` | `PathAlignCritic::score` | 0.53 ms | 0.036 ms | 15× | 2.3e-5 |
| `obstacles_critic` (footprint) | `ObstaclesCritic::score` (consider_footprint=true) | 0.13 ms | 0.034 ms | 4× | **0** (bit-exact) |
| `cost_shape` | MPPI bias term in `updateControlSequence` | 0.05 ms | 0.014 ms | 4× | 7.6e-6 |
| `softmax` | softmax over costs[B] | 0.027 ms | 0.014 ms | 2× | 1.5e-8 |
| `weighted_avg` | weighted sum → control sequence | 0.049 ms | 0.016 ms | 3× | 1.5e-8 |
| **TOTAL pipeline** | `Optimizer::optimize()` body | **~12 ms** | **~0.25 ms** | **~50×** (RTX 4050) | — |

- **`ObstaclesCritic` is fully footprint-aware** (`consider_footprint: true` mode, matching our `nav2_go2w_full_stack.yaml`): per-pose footprint rotated + rasterised via Bresenham over the GPU costmap, max cost along each polygon edge. Bit-identical to CPU because the costmap is uint8, worldToMap is integer truncation, and the Bresenham line walks identical cell sets (already cell-traced against `nav2_util::LineIterator` in the compat shim).
- **Orin NX projection**: RTX 4050 GPU ~3× stronger than Orin NX Ampere; Ryzen 9 CPU ~5× stronger than 6×A78AE. So GPU latency on Orin extrapolates to ~0.75 ms total pipeline, CPU latency extrapolates to ~60 ms — net **~80×** end-to-end on the real deployment target. To be confirmed by an actual Orin run; this is back-of-envelope from per-kernel numbers and architectural ratios.
- **Together with CFPA2 C++ port**: full autonomy stack (frontier allocator + nav planner + controller) is now Noetic-deployable. CFPA2 hexagonal-isolation route + Nav2 compat-shim route are independent strategies for the same end goal (Noetic onboard) — both ship.

### Strategy: compat shim, NOT rewrite

Two ways to "port Nav2 to ROS 1": (a) write equivalent algorithms from scratch using sbpl + MPPI-Generic (~4-7 weeks), or (b) lift the algorithm-body source verbatim and bridge the ROS-2-specific surface with a 380-line compat header. **(b) is what shipped.** Picked because the algorithm code is mostly ROS-agnostic templated C++ over `xtensor` + `Eigen` + `OMPL` + `nlohmann::json`, and Nav2's `nav2_costmap_2d::Costmap2D` was itself forked from ROS 1's `costmap_2d::Costmap2D` — same data structures, same accessors. The actual ROS-coupled surface is concentrated in: rclcpp logging, lifecycle node param API, and ROS 2 message types (`*::msg::*` vs ROS 1's bare type). All three can be shimmed.

### 5 catkin packages under `src/vendor/nav_algo_ros1/`

| Package | Role | Output |
|---|---|---|
| `nav_algo_core` | Algorithm-only static library (Smac + MPPI bodies + compat shim + custom param facade) | `libnav_algo_smac.a` 1.9 MB + `libnav_algo_mppi.a` 3.1 MB |
| `nav_algo_smac_ros1` | `nav_core::BaseGlobalPlanner` plugin wrapping `AStarAlgorithm<NodeLattice>` | `libnav_algo_smac_ros1.so` + plugin.xml |
| `nav_algo_mppi_ros1` | `nav_core::BaseLocalPlanner` plugin wrapping `mppi::Optimizer` + 8 critics | `libnav_algo_mppi_ros1.so` + plugin.xml |
| `nav_algo_bringup` | `move_base` launch + 5 yaml configs (costmap common/global/local + smac + mppi) | runtime artifacts |
| `nav_algo_mppi_cuda` | First CUDA kernel (`integrateStateVelocities`) + CPU-vs-GPU bench + diff | `libnav_algo_mppi_cuda.so` + `integrate_bench` executable |

### compat.hpp (the 380-line bridge)

Coverage of every ROS 2 surface the algorithm body touches:

| Nav2 source uses | compat.hpp provides |
|---|---|
| `RCLCPP_INFO(logger, ...)` and 4 sibling levels | Macros that drop the logger arg, route to `ROS_INFO(...)` |
| `RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...)` | Drop logger+clock, convert ms→s, route to `ROS_ERROR_THROTTLE` |
| `geometry_msgs::msg::PoseStamped` (and 9 sibling types) | `using PoseStamped = ::geometry_msgs::PoseStamped` in `geometry_msgs::msg` ns |
| `nav2_costmap_2d::{Costmap2D, Costmap2DROS, InflationLayer, Layer, Footprint, FREE_SPACE, LETHAL_OBSTACLE, ...}` | Re-export via `nav2_costmap_2d = costmap_2d` namespace alias + `using` for each type |
| `nav2_costmap_2d::FootprintCollisionChecker<T>` (Nav2-only template) | Re-implemented inline (~70-line Bresenham, traced cell-by-cell against upstream) |
| `rclcpp_lifecycle::LifecycleNode::SharedPtr parent` | Stub class that wraps a `ros::NodeHandle`; provides `get_logger / now / get_clock / get_parameter<T>` |
| `rclcpp::Time / Duration / Clock / Parameter / ParameterValue` | POD stubs (Time has implicit conversion to/from `ros::Time`) |
| `nav2_util::declare_parameter_if_not_declared(node, name, ParameterValue(def))` | No-op stub (ROS 1 rosparam is implicitly typed) |
| `nav2_util::geometry_utils::{euclidean_distance, orientationAroundZAxis, first_after_integrated_distance, min_by}` | Inline re-implementations (traced against upstream for the two non-trivial ones) |
| `nav2_core::PlannerException / GoalChecker` | Local subclass of `std::runtime_error` + abstract interface (`getTolerances`) |

`ParametersHandler` was the only file that needed full rewrite (not shim): Nav2 reads params via rclcpp's declare/get; we substitute a `getParamGetter(ns)` returning the same lambda shape but reading rosparam under `ros::NodeHandle` (with `"."` → `"/"` separator translation). Every critic / optimizer / handler call site compiles unchanged.

### Surgical patches in 5 files (NOT math changes)

- **`obstacles_critic.cpp`**: dropped `costmap_ros_->getUseRadius()` mismatch warn+throw (ROS 1 Costmap2DROS has no such introspection); `std::dynamic_pointer_cast<InflationLayer>` → `boost::` (ROS 1 layer plugins use `boost::shared_ptr`).
- **`collision_checker.cpp` / `node_hybrid.cpp`**: `Costmap2D::getCost(idx)` single-arg overload doesn't exist in ROS 1 → `getCharMap()[idx]` (bit-identical).
- **`smac/utils.hpp`**: iterator type `std::shared_ptr<Layer>` → `boost::shared_ptr<Layer>` for the same reason.
- **`path_handler.cpp`**: `tf2::durationFromSec(t)` → `ros::Duration(t)` (ROS 1's `tf2_ros::Buffer::transform` accepts ros::Duration directly).
- **`costmap_downsampler.cpp`**: stubbed `Costmap2DPublisher` ctor (ROS 1 signature differs; yaml has `downsample_costmap: false` so the publisher path is unreachable anyway). Dropped `on_activate / on_deactivate` calls (Nav2-lifecycle-only).

Each documented inline with the rationale. None affect cmd_vel under valid yaml.

### Equivalence audit (before integration test)

| Change | Verification | Risk |
|---|---|---|
| Mass include rename (60 files) | sed-only, no semantic edits | zero |
| compat.hpp logging/msg shims | logger is side-effect-only; ROS 1 vs ROS 2 msg field names are identical | zero |
| `nav2_costmap_2d::*` → `costmap_2d::*` alias | same fork, same API surface (verified Costmap2D::worldToMap / getCost / getResolution / etc.) | zero |
| compat.hpp `FootprintCollisionChecker` re-impl | Traced (0,0)→(4,2) and (0,0)→(2,4) lines cell-by-cell against Nav2's Bresenham `LineIterator`: same cell sets visited | zero (after trace) |
| compat.hpp `geometry_utils::first_after_integrated_distance` re-impl | Traced N=5 poses, distances `[1, 1.5, 0.3, 0.4]`, target 2.0 → both return `begin+2` | zero |
| compat.hpp `geometry_utils::min_by` re-impl | Same init+loop shape, pass-by-value iter | zero |
| `parameters_handler` rewrite | Lambda signature preserved (`(setting&, name, default, type)`); all critics + optimizer call sites unchanged | low (no dynamic reconfigure under ROS 1; static-load only) |
| Surgical patches (obstacles_critic etc.) | Each only affects misconfig path OR uses bit-identical accessor | low |
| Algorithm cpp body | `diff` shows only include line + compat.hpp injection per file | zero |

Conclusion: **for valid yaml + valid input**, ROS 1 port cmd_vel **math = Nav2 Humble cmd_vel**. Only sources of numerical divergence on the integration test would be: (a) random-seed mismatch in `xt::random` (we keep xtensor 0.24.7 to match Humble), (b) reduction reordering on GPU softmax (deferred to Orin validation).

### CUDA kernel design patterns

All 11 kernels follow one of three patterns, picked per problem shape:

- **Per-trajectory reduction** (10 of the 11 critics + cost_shape): 1 CUDA block per trajectory, T threads (kThreadsPerBlock=64 covers up to T=56 Nav2 canonical), each thread computes its (b, t) contribution, then `cub::BlockReduce::Sum` collapses the T-wide row to one number that thread 0 emits via `costs[b] +=`. No inter-block sync; no atomics.
- **B-wide reduction** (softmax): 1 block of 256 threads, grid-stride loop over B = 2000. Three phases (min-reduce → exp+accumulate → normalize) all inside the same kernel via `__shared__` broadcast variables and `__syncthreads()`. Defensive uniform-fallback when `sum_exp == 0`.
- **Per-column scatter** (weighted_avg): T blocks × 256 threads, each block sums B-wide for one t. One launch per dimension (vx, vy, wz).

The `integrate` kernel additionally uses `cub::BlockScan::InclusiveSum` to do the cumsum over `wz·dt` (yaws) and over `dx·dt`/`dy·dt` (x, y positions) in parallel within each trajectory block — collapses a sequential cumsum across T into log2(T) parallel steps.

`ObstaclesCritic` has an inner inline `__device__ footprintCostAtPoseGpu()` helper that rotates the polygon vertices, calls `worldToMapGpu`, and runs Bresenham line-walks along each edge. The line walk is the Wikipedia error-accumulator variant; bit-identical to `nav2_util::LineIterator` (Willow Garage Bresenham) — both forms visit the same cell sets (verified by hand-tracing (0,0)→(4,2) and (0,0)→(2,4) in the compat shim audit).

Per-trajectory complexity:
```
B=2000, T=56 (Nav2 canonical shape on Go2W)
hardware: RTX 4050 mobile, sm_89, ~1.8 TFLOPS fp32

integrate single-kernel call : 0.016 ms (median of 5)
ObstaclesCritic (footprint)  : 0.034 ms — includes ~40 cell lookups per pose
                                          across 4 footprint edges × Bresenham
PathAlignCritic              : 0.036 ms — heaviest critic, per-thread binary
                                          search on path_integrated_distances
8 critics + cost_shape +     : ~0.25 ms total sequential on the same stream
softmax + weighted_avg
```

Build flags: `-gencode arch=compute_87,sm_87` (Orin Ampere) `+ -gencode arch=compute_89,sm_89` (laptop RTX 4050) so the same `.so` runs on both. Cached Docker image `nav_algo:build_env_cuda` (3 GB) bakes in CUDA 12.6 toolkit + xtensor 0.24.7 + nlohmann-json + nav_core for one-command catkin builds.

### Optimizer integration — `ICudaBackend` injection

Clean separation between `nav_algo_core` (no CUDA at any build level) and `nav_algo_mppi_cuda` (CUDA-only). Done via dependency-injection:

1. **[`nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp`](src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp)** defines `mppi::ICudaBackend` — pure-virtual `optimize(Optimizer&)`. Zero CUDA headers; compiles on a CUDA-less dev machine. `nav_algo_core` continues to build with no GPU toolchain available.
2. **`mppi::Optimizer`** holds an `ICudaBackend* cuda_backend_` (default `nullptr`). The hot loop at `optimizer.cpp:155` becomes:

```cpp
void Optimizer::optimize() {
  if (cuda_backend_ != nullptr) {
    cuda_backend_->optimize(*this);
    return;
  }
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
  }
}
```

3. **`Optimizer` exposes minimal public accessors** (state, control_sequence, generated_trajectories, path, costs, settings, critic_manager, critics_data, motion_model) so the backend reads/writes them per cycle without becoming a `friend` class. Also adds `generateNoisedTrajectoriesNoIntegrate()` so the backend can run CPU noise + motion model but skip the CPU integrate (GPU does its own integrate kernel).
4. **[`nav_algo_mppi_cuda::CudaBackend`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/include/nav_algo_mppi_cuda/cuda_backend.hpp)** is the concrete impl. Owns all persistent device buffers sized at ctor (`CudaBackendConfig` with B, T, P_max, costmap_max_cells, footprint_max_n). One `optimize()` per cycle does H2D (state+control+costmap+path) → 11-kernel chain on the default stream → D2H (control_sequence) → host-side constraint clamp.
5. **MPPI plugin** ([`nav_algo_mppi_ros1`](src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/src/mppi_controller_ros.cpp)) reads `use_cuda` yaml param; when `true` AND `NAV_ALGO_MPPI_HAS_CUDA` is defined at build time (auto-set when `nav_algo_mppi_cuda` is in the workspace), instantiates `CudaBackend`, uploads footprint, attaches via `optimizer_.setCudaBackend(...)`. When CUDA not built but yaml requests it: `ROS_WARN` + silent CPU fallback.
6. **Verification via probe files** (cuda_backend.cu `/tmp/cuda_backend_{ctor,optimize}` instrumentation, retained as cheap operational monitoring): integration test recorded 200+ `Optimizer::optimize()` dispatches in 8 s, ~25 Hz, all going through `CudaBackend` (xtensor CPU path silent).

### v1 known limitations (each a follow-up commit)

- **Host-side critic gates are skipped.** `within_position_goal_tolerance`, `posePointAngle`, `max_path_occupancy_ratio` checks — these gate CPU critics off near goal / on misaligned headings / when path is blocked. v1 backend runs every critic every cycle. Math diverges from Nav2 sim in the near-goal / path-corrupt cases; far from goal the cost contributions collapse correctly (tested in bench cases). v2: add scalar precomputation on host before each kernel launch.
- **`PathAlignCritic` uses `furthest_reached = path_size`.** CPU computes a robot-pose-dependent smaller index; we currently use the full path → cost contribution slightly conservative. v2: compute on host before launch.
- **Critic weights HARDCODED in `cuda_backend.cu`** (values copied from `nav2_go2w_full_stack.yaml`: ConstraintCritic 4.0, ObstaclesCritic critical 20 / repulsion 3, PathAlign 14, etc.). Yaml edits don't propagate until `CudaBackend` is rebuilt. v2: read from `CriticManager` via accessor.
- **Footprint set once at plugin init.** Dynamic footprint changes mid-run aren't re-uploaded. v2: re-poll `costmap_ros_->getRobotFootprint()` per cycle, dirty-check, re-upload only on change.

### Build env caveats

`ros:noetic-ros-core` Docker (Ubuntu 20.04 + CMake 3.16) needs three additions beyond apt: (1) xtensor + xtl from source (apt has no `libxtensor-dev` on focal), (2) CUDA 12.6 toolkit via the NVIDIA apt repo, (3) `nlohmann-json3-dev`. CMake 3.16 can't use `find_package(CUDAToolkit)` (that's 3.17+) so we `find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64)` instead, and skip `CMAKE_CUDA_ARCHITECTURES` (3.18+) in favor of inline `-gencode` flags. `add_compile_options(-Wall)` MUST be gated with `$<$<COMPILE_LANGUAGE:CXX>:...>` or nvcc errors on `-Wall` (it expects `-Xcompiler -Wall`). The cached `nav_algo:build_env_cuda` image encodes all of these.

### Files / paths

- [`src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/compat.hpp`](src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/compat.hpp) — 380-line shim
- [`src/vendor/nav_algo_ros1/nav_algo_core/src/{smac,mppi}/`](src/vendor/nav_algo_ros1/nav_algo_core/src/) — 60 source files lifted from Nav2 (verbatim minus include rewrite)
- [`src/vendor/nav_algo_ros1/nav_algo_smac_ros1/`](src/vendor/nav_algo_ros1/nav_algo_smac_ros1/) — global planner plugin + 5cm/0.5m diff lattice JSON (4876-line file shipped, lifted from `/opt/ros/humble/share/nav2_smac_planner/`)
- [`src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/`](src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/) — local planner plugin
- [`src/vendor/nav_algo_ros1/nav_algo_bringup/{launch,config,test}/`](src/vendor/nav_algo_ros1/nav_algo_bringup/) — `move_base.launch` + 5 yaml + `integration_test.sh`
- [`src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/) — first kernel + bench
- [`scripts/bench/record_nav2_replay_bag.sh`](scripts/bench/record_nav2_replay_bag.sh) — captures sim bag (cmd_vel + costmap + plan + tf) as gold input for future equivalence replays once full plugin chain runs on real bags

### Open / next

- **v2 critic gates** (the math-equivalence finishing touch). All 11 kernels are validated against CPU references in isolation but the *backend driver* currently runs every critic every cycle (no host-side gates). For correctness on a real scene with goal-tolerance regions, near-goal poses, and partially blocked paths, the host-side scalar checks need to be wired in: `within_position_goal_tolerance`, `posePointAngle`, `furthest_reached_path_point`, `max_path_occupancy_ratio`. Each is a small precompute → conditional kernel launch.
- **yaml-driven critic weights**. `CudaBackend::optimize()` currently hardcodes the numbers from `nav2_go2w_full_stack.yaml`. Wire through `CriticManager` accessors so yaml edits propagate without a rebuild.
- **Equivalence replay test**: `record_nav2_replay_bag.sh` is in place but the actual cross-version replay tooling (rosbag2 mcap → rosbag1 conversion + plugin-driven replay through our move_base) isn't built yet. Useful once integration test moves from synthetic OccupancyGrid to actual sim-recorded inputs. This is what would conclusively prove cmd_vel ≈ Nav2 sim cmd_vel.
- **Orin NX onboard deployment** is now unblocked. The full GPU pipeline is integration-tested on RTX 4050; cross-compile or rebuild the same packages on the Jetson, deploy alongside Point-LIO / `trav_pipeline_ros1` / gbplanner3 (all already Noetic-native). Need: a JetPack-6-targeted Docker build env or in-tree Jetson catkin workspace; sm_87 already included in `-gencode` flags.
- **Smoke test on real Noetic** (vs Docker): the cached image is Ubuntu 20.04 Focal; Jetson JetPack 6 uses Ubuntu 22.04 Jammy with ROS 1 Noetic backported by the user (`~/noetic_fastlio_ws/` etc.). Build env may need tweaks; planned as part of the Orin onboard step.

## Active state (2026-05-19) — CFPA2 pure C++ port + hexagonal isolation for ROS 1/2 portability

Took CFPA2 from Python-with-ctypes-accelerator to a fully-C++ `rclcpp::Node` with the algorithm body completely decoupled from ROS so a future Noetic port is "swap one adapter directory + edit the ctor". Net 2,100 lines deleted / 2,400 added (51 files changed) in commit `d0526bf`.

### TL;DR

- **Jetson aarch64 tick p95 = 1.1 ms** (vs Python's 1376 ms historic — **~1250× speedup**). Adaptive load shedding is now dead-code: tick is ~500× under the 500 ms budget on Orin Nano 8 GB, so stride/targets/gain_r/skip downshifts never need to kick in.
- Same hot algorithm running unmodified on desktop x86 (tick p95 = 0.6 ms) and Jetson aarch64 — single binary, native rclcpp, no ctypes / numpy / scipy in the production data path.
- **Algorithm body has zero ROS-specific calls.** All `RCLCPP_*` / `get_clock()->now()` / `nav_msgs::msg::*` / `rclcpp::Publisher<T>::publish` accesses route through abstract interfaces in [`include/cfpa2_collaborative_autonomy/core/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/core/). The Noetic port = write `ros1/*.hpp` adapters that wrap `ros::Time` / `ROS_INFO` / `nh.subscribe` / etc., plus a ctor edit. Full diff guide: [docs/claude/noetic_port_checklist.md](docs/claude/noetic_port_checklist.md) (estimated ~5 h end-to-end).
- Two binaries installed side-by-side with the Python entry points (`cfpa2_*_node` Python / `cfpa2_*_node_cpp` C++). Flip via launch arg `cfpa2_executable_suffix:=_cpp` — threaded through `navigation.launch.py`, `single_go2w_mujoco_cfpa2.launch.py`, `nav_test_mujoco_fastlio.launch.py`, `nav_test_3d_explore.launch.py`.

### Strip first — kill dead code before porting

Before any C++ work, audit + grep found 1,250 LOC of dead Python paths:
- **3D `ig_dimension` path** ([`frontier_3d.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py) + [`cluster_tracker.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cluster_tracker.py) + `frontier_3d_test_node.py` + `cfpa2_single_robot_3d.yaml` + `nvblox_frontend_msgs` dep) — zero production launches enabled 3D mode; `nav_test_3d_explore.sh` defaulted to 2D since the elevation_mapping_cupy pipeline landed.
- **`shared_map` multi-robot fusion** ([`map_merge_utils.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/map_merge_utils.py) + `use_shared_map` params + `_build_fallback_map` / `_build_shared_with_local_patches`) — single_robot subclass explicitly overrode it to `false`, and no production launch ever set `use_shared_map=true`. The PR-4 decentralised peer-coordination (ylhaichen) replaces this entirely via `/<ns>/cfpa2_peer_coordination/blocked_frontiers`.
- **`algorithm_mode` / `output_mode` literal-only attrs** — mtare / mui_tare / committed paths were removed 2026-05-10; the literal strings were kept only for log formatting and got stripped here.
- **`cfpa2_planner_mode="greedy"` fork** — production always used `tsp_topk`; the greedy fallback path was dead and got removed.
- **`setup.py` / `setup.cfg`** — orphans after the ament_cmake migration.
- **`peer_map_merger_node.py`** — empty stub in the PR-4 package (peer_map_merger explicitly out of scope per ylhaichen).

Same-commit also dropped the bundled-with-PR-4 `peer_map_merger` entry from the peer coordination README so docs match the deletion.

### Phase A — modular `ops/` kernel library

Split the 553-LOC monolithic `cfpa2_grid_ops.cpp` into per-function headers + sources in [`include/cfpa2_collaborative_autonomy/ops/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/ops/) + [`src/ops/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/ops/):
- `extract_frontiers`, `distance_transform[_range]`, `batch_info_gain`, `batch_info_gain_floodfill`, `filter_dead_frontiers`, `cluster_representatives`, plus shared `grid_offsets.hpp` (DX4/DY4/DX8/DY8 constants)
- All under namespace `cfpa2::ops::*` — no ROS deps.
- [`src/cfpa2_grid_ops_c_api.cpp`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_grid_ops_c_api.cpp) is the `extern "C"` ABI shim so the Python coordinator's ctypes loader keeps working during the transition.

### Phase B — pure C++ rclcpp Nodes

New per [`cfpa2_collaborative_autonomy/CMakeLists.txt`](src/collaborative_exploration/cfpa2_collaborative_autonomy/CMakeLists.txt):

| Target | Type | Contents |
|---|---|---|
| `cfpa2_ops` | STATIC | Kernel library; pure `cfpa2::ops::*` (no ROS). |
| `cfpa2_grid_ops` | SHARED (.so) | `extern "C"` ABI shim for Python ctypes. |
| `cfpa2_node_lib` | STATIC | Coordinator + single_robot algorithm bodies. |
| `cfpa2_coordinator_node_cpp` | executable | Dual-robot joint-allocator binary. |
| `cfpa2_single_robot_node_cpp` | executable | Single-robot binary (production path). |

`CFPA2Coordinator` lives in [`include/cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp) + [`src/cfpa2_coordinator.cpp`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_coordinator.cpp); `CFPA2SingleRobotNode` is the subclass in [`cfpa2_single_robot.{hpp,cpp}`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/cfpa2_single_robot.hpp).

### Phase C — full goal-selection state machine

- **`apply_goal_policy`** (faithful port of Python's 150-LOC `_apply_goal_policy`): stranded-frontier check (held goal not within `cfpa2_stale_frontier_radius_m` of any current frontier → blacklist + force switch); stable-challenger override (same candidate top-1 for `challenger_streak_required` ticks AND `challenger_min_lock_age_sec` elapsed AND score beats held × `challenger_improvement_factor` → preempt); `goal_lock_sec` min-hold; `switch_min_dist` hysteresis; progress-vs-stalled hold; `safety_failure` (unreachable / unsafe_clearance) → register failure + switch.
- **`apply_fast_blacklist`**: manual lightweight JSON parser on `nav_status` payload (no `nlohmann` dep), match reported goal to `last_goal` by quantised key, `fast_unreachable_startup_grace_sec` (15 s default) + `fast_unreachable_consecutive_threshold` (3-emit debounce), latch 60 s blacklist + cluster disk + reset fail counters + dedup by `goal_seq`.
- **Joint allocator (dual-robot path)**: when `namespaces_.size() == 2`, iterate every (goal_a, goal_b) pair from the per-ns utility lists, score `joint = (u_a + u_b) × clamp(1 − λ·overlap(a,b), 0, 1)`, pick the max non-equivalent pair, apply per-ns goal policy on top. Single-robot path falls through to TSP-top-K head.

### Phase D/E — hexagonal isolation (the Noetic-port enabler)

The algorithm body now has **zero ROS-specific calls.** Everything goes through abstract interfaces in [`include/cfpa2_collaborative_autonomy/core/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/core/):

| Header | Purpose |
|---|---|
| `core/types.hpp` | POD `Grid`, `GridInfo`, `OdomXY`, `Goal`, `GoalKey`, `ScoredGoal`, `BlacklistDisk`, `PoseSample`, `ProgressSample`, `UtilityList` |
| `core/clock.hpp` | `IClock::now_ns()` abstract interface |
| `core/logger.hpp` | `ILogger::info/warn/error` abstract |
| `core/logging.hpp` | `CFPA2_LOG_INFO/WARN/ERROR` printf-style macros that route through `ILogger` |
| `core/output.hpp` | `IGoalPublisher` (publish_goal / publish_goal_marker / publish_status) + `IVisualizer` (publish_coordinator_map / publish_robot_markers / publish_frontier_markers) + supporting `TrajectoryView` / `RobotPoseView` PODs |

ROS 2 adapter implementations in [`include/cfpa2_collaborative_autonomy/ros2/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/ros2/) — all header-only:

| File | Implements |
|---|---|
| `rclcpp_clock.hpp` | `IClock` via `rclcpp::Clock::SharedPtr` |
| `rclcpp_logger.hpp` | `ILogger` via `RCLCPP_INFO/WARN/ERROR` macros |
| `conversions.hpp` | `to_core_grid` / `to_msg_grid` / `to_core_odom` / `to_msg_point_stamped` (boundary marshallers) |
| `rclcpp_goal_publisher.hpp` | `IGoalPublisher` wrapping `rclcpp::Publisher<PointStamped/Marker/String>` |
| `rclcpp_visualizer.hpp` | `IVisualizer` wrapping `rclcpp::Publisher<OccupancyGrid/MarkerArray>` |

After Phase D/E the algorithm body's ROS contact surface is **only**: `rclcpp::Node` inheritance + `declare_parameter` / `get_parameter` + `create_subscription<T>` + `create_wall_timer` (all in the ctor). Everything else is interface calls. The Noetic port replaces those ~5 ctor entries + writes 5 `ros1/*.hpp` adapter headers; algorithm body (1,400 LOC) requires zero hand edits.

### Goal-jitter fixes (caught during sim test)

Live sim with C++ binary showed goal oscillating between two frontiers (4.95, 0.45) and (8.15, 0.45) at ~5 Hz, triggering Nav2's BackUp recovery loop. Three independent bugs:

1. **`tsp_top_k_head` was a simplified "nearest of top-K to robot"** — equivalent to the Python `first_idx` semantics but not the literal port. Replaced with the full NN-tour faithful port. Doesn't change behavior but is more defensible against future K-tour refinements.
2. **`std::sort` on the per-ns utility list was unstable** — near-tie utilities flipped top-K order tick-to-tick → TSP head flipped → goal oscillation. Fixed to `std::stable_sort` to match Python's `sorted(...)` stability guarantee.
3. **`set_active_goal` was overwriting `last_goal_set_time_ns_` unconditionally** — the **real root cause**. Python's version guards with `if math.hypot(prev - goal) > 1e-6:`. Without that guard the lock-age clock resets every tick → never exceeds the tick interval → **every time-based gate in `apply_goal_policy` (goal_lock_sec / challenger_min_lock_age_sec / stuck_lock_sec) is inert**. Fixed to match Python: only reset the clock when the goal actually changes.

### Performance comparison

Same synthetic bench (200×200 grid, 6 frontier corridors, 5 Hz publish_rate, full pipeline: extract → cluster → filter → IG → utility → publish):

| | tick p50 | tick p95 | tick max | goals / 27 s |
|---|---|---|---|---|
| Python coordinator on Jetson (historic, 2026-05-18) | ~400 ms | **1376 ms** ❌ | — | — |
| **C++ port on desktop x86** | **0.4 ms** | **0.6 ms** | 0.7 ms | 1073 |
| **C++ port on Jetson aarch64 (Orin Nano 8GB)** | **1.0 ms** | **1.1 ms** | 1.4 ms | 1213 |

Jetson C++ p95 is **1250× faster** than the Python coord on the same hardware, and **500× under** the 500 ms budget. The adaptive load-shedding parameters (`adaptive_max_frontier_stride`, `adaptive_min_max_targets`, `adaptive_min_exploration_gain_radius_cells`, `adaptive_max_skip_ticks`) become dead-code on the C++ path — tick never gets near the budget where downshifts would fire.

### Sim integration

End-to-end verified on desktop via `./scripts/launch/nav_test_3d_explore.sh cfpa2_executable_suffix:=_cpp` — C++ binary loads both `cfpa2_single_robot.yaml` (base) + `cfpa2_single_robot_demo_ramp.yaml` (overlay) correctly, subscribes / publishes every topic the Python entry point did, perf logs stay deep inside budget. Two downstream issues observed (NOT C++ port bugs — Python coordinator on same machine has the same failures):
- **Nav2 `bt_navigator` XML load error** → `unknown goal response, ignoring...` flood → `cfpa2_to_nav2_bridge` fast-BL bursts → tight goal-cancel race. Unrelated to this port.
- **`grid_map_to_occupancy_grid` projects robot-adjacent cells as walls** in demo_ramp scene (likely an elevation_mapping_cupy projection bug or self-filter mis-tuning). Unrelated to this port.

When Nav2 BT fails to follow a goal and the robot doesn't move for `stuck_watchdog`'s 10 s window, the watchdog fires Nav2's `BackUp` behavior — that's the "robot keeps backing up" behaviour the user observed; root cause is the Nav2 BT bug, not CFPA2. Fix-or-bypass for the Nav2 BT issue is a separate task.

### Files / paths

- New header tree: [`include/cfpa2_collaborative_autonomy/{core,ops,ros2}/`](src/collaborative_exploration/cfpa2_collaborative_autonomy/include/cfpa2_collaborative_autonomy/)
- New source tree: [`src/{ops/,cfpa2_coordinator.cpp,cfpa2_single_robot.cpp,*_node_main.cpp,cfpa2_grid_ops_c_api.cpp}`](src/collaborative_exploration/cfpa2_collaborative_autonomy/src/)
- New CMakeLists: [`CMakeLists.txt`](src/collaborative_exploration/cfpa2_collaborative_autonomy/CMakeLists.txt) — ament_cmake (was ament_python)
- New package.xml: [`package.xml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/package.xml) — drops nvblox_frontend_msgs dep, adds rclcpp / tf2 / visualization_msgs / cfpa2_peer_coordination_msgs
- Launch arg `cfpa2_executable_suffix` plumbed through [`navigation.launch.py`](src/go2w/go2w_config/launch/navigation.launch.py), [`single_go2w_mujoco_cfpa2.launch.py`](src/go2w/go2_gazebo_sim/launch/single_go2w_mujoco_cfpa2.launch.py), [`nav_test_mujoco_fastlio.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py), [`nav_test_3d_explore.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py).
- Noetic port guide: [`docs/claude/noetic_port_checklist.md`](docs/claude/noetic_port_checklist.md) — file-by-file diff with sed-style mechanical changes + estimated work breakdown (~5 h end-to-end including build + Orin NX smoke).
- Updated [`scripts/real/deploy_to_orin_nano.sh`](scripts/real/deploy_to_orin_nano.sh) to sync `cfpa2_peer_coordination[_msgs]` (was just `cfpa2_collaborative_autonomy`).

### Open / next

- **E-4 deferred**: full split of `CFPA2Coordinator` into `core::Coordinator` (pure algorithm, no `rclcpp::Node` inheritance) + `ros2::CoordinatorNode` (the `rclcpp::Node` wrapper that owns a `core::Coordinator` instance). Current state has algorithm body completely ROS-agnostic but the *class itself* still inherits `rclcpp::Node`. Full split would cut the Noetic port ctor work further (5 h → ~3 h) but is a 1.5-2 h refactor that wasn't worth the risk vs the marginal benefit. LLM-driven Noetic port from current state is the recommended path.
- **No separate C++ test suite yet**: the 36-test Python pytest suite exercises Python coordinator logic and all still pass. The C++ port has been verified by perf bench + sim integration but doesn't have its own gtest yet. Future work.
- **Nav2 `bt_navigator` XML load failure** (independent of CFPA2 port) is the next thing to fix for end-to-end sim demos to actually drive the robot. Likely a Humble dist artifact or BT XML path resolution issue.

## Active state (2026-05-18 late night) — Jetson Orin Nano 8GB HIL bag-replay: fast_lio real-time confirmed, Point-LIO ROS 2 port dead-ended

**Goal**: prove the full autonomy stack (SLAM + elevation + Nav2 + CFPA2) sustains real-time on Orin Nano 8GB (`johnpork233@192.168.55.49`, JetPack 6.2.2, 6× Cortex-A78AE @ 1.5 GHz, 8 GB RAM) **as a weaker-board proxy for the real Go2's Orin NX 16GB** (8× @ 2.0 GHz, 16 GB). Pipeline must keep up under wall-clock bag-replay (`onboard_noetic_20260511_155920_ops2_ros2_raw`, 180s real Go2 walk, raw `/livox/imu` 200 Hz + `/livox/lidar` 10 Hz CustomMsg only — SLAM rebuilt from scratch, no pre-recorded Odometry / TF used).

### TL;DR

- **fast_lio (HKU FAST-LIO2 = `src/vendor/fast_lio`) is the production SLAM**. On bench Orin Nano: `/robot/Odometry` 10 Hz for 170s, RSS 192 MB constant (ikd-tree NOT unbounded with our config), CPU 9-12% single-core. RTF measured = 1.07 (sim/wall within ±5% noise). Real RTF = 1.0. Pose makes physical sense (~1 m/s walking, matches the captured Odometry baseline trajectory).
- **dfloreaa's Point-LIO ROS 2 port (`src/vendor/point_lio_ros2`) is abandoned for this hardware.** Two independent show-stoppers:
  1. **Heisenbug SIGSEGV** at IMU Init 100% (exit -11) on aarch64, ~50% reproduction rate in full-launch. Standalone gdb doesn't reproduce. Cause: startup race between Point-LIO's first publish, bag's first IMU/lidar arrival, and DDS publisher-subscriber matching window. Respawn=True works (2nd-Nth respawn succeeds because subscribers already discovered), but only buys process liveness — see #2.
  2. **SLAM divergence even when alive**: pose accelerates to (-148, -72, -33) in 6 seconds (40 m/s implied velocity, robot was walking 1 m/s). Tuned `mid360_go2_real.yaml` (`start_in_aggressive_motion: true`, `satu_acc 3→5`) reduced divergence rate ~30× but did not fix it. Same IMU+lidar data → fast_lio gives sensible (~1 m/s) trajectory.
- **Real bottleneck on Jetson is NOT SLAM. It's the Python single-core nodes on ARM**:
  - **CFPA2 single_robot** tick p95 **1376 ms vs budget 500 ms** (2.75× over). Adaptive load shedding kicked in (stride 2→8, targets 180→80, gain_r 40→27, skip 0→3). Still usable, but frontier replan responds at ~1.4s instead of 100ms.
  - **grid_map_to_occupancy_grid** publishes `/robot/traversability_grid` at **0.59 Hz** (vs ~5 Hz target). Single-core Python on ARM at 99% CPU.
  - Both will not improve on Orin NX 16GB — same single-core perf ratio (1.33× = ~1s tick at best). Long-term fix: numba JIT hot path or partial C++ rewrite.

> **→ Resolved 2026-05-19** by full C++ port of CFPA2 (see "Active state (2026-05-19)" above). Jetson tick p95 dropped 1376 ms → 1.1 ms (1250× speedup). Single-core Python on aarch64 is no longer the bottleneck — adaptive load shedding becomes dead-code.

### Origin / record bag context

The bag was recorded onboard the real Go2 (Orin NX 16GB) on 2026-05-11 with Mid-360 + IMU. SLAM stack at the time was **Noetic FAST-LIO** → ros1_bridge → ROS 2 → `rosbag record`. The recorded `/robot/Odometry` has Count=1023 over 180s = **5.7 Hz average**. This was originally cited (CLAUDE.md 2026-05-13 archive note) as "FAST-LIO2 degraded from 9 Hz to 4.4 Hz over 3-min ops2 walk: ikd-tree grew unbounded". This bench test contradicts that conclusion — see "Why bench is faster than onboard" below.

### What got it working — the 8 fixes shipped today

1. **DDS isolation via `ROS_DOMAIN_ID=42`** in `run_jetson_bag_full_load.sh`. Cross-host multicast was leaking ghost `imu→body` static-TF publishers from the desktop's HIL stack into the Jetson's TF tree, splitting it into `{map→odom}` + `{imu→body→base_link}` two unconnected trees ("Could not find a connection between odom and base_link"). Even after the desktop HIL stack was killed, the DDS discovery cache held the phantoms for tens of minutes. Domain isolation is the surgical fix.
2. **`livox_ros_driver2_msgs` minimal msgs-only package** at `src/vendor/livox_ros_driver2_msgs/` (separate dir, COLCON_IGNORE the original `src/vendor/livox_ros_driver2/`). Provides `livox_ros_driver2/msg/CustomMsg` + `CustomPoint` deserialization for bag-replay without building the SDK or driver binary on the Jetson. `package.xml` declares the package as `livox_ros_driver2` (same name → fast_lio's `find_package(livox_ros_driver2 QUIET)` finds it). Required also: rebuild fast_lio so the conditional `livox_ros_driver2_FOUND` path compiles in (a previous fast_lio binary built without it threw "Livox lidar_type selected but livox_ros_driver2 not available" at runtime).
3. **Bag-play preflight env sourcing**: launching `ros2 bag play` without `source /home/johnpork233/jetson_ws/install/setup.bash` first → `rosbag2_player` can't find `livox_ros_driver2/msg/CustomMsg` → silently drops the `/livox/lidar` topic with a single WARN line. Symptom: `/livox/imu` flows at 200 Hz but `/livox/lidar` publisher_count = 0. Fix: workspace source in every bag-play invocation.
4. **Bag-play `--clock` + `use_sim_time=true`** everywhere downstream. Without `--clock`, bag's message timestamps (recorded 2026-05-11) appear "1 week stale" to `ros::Time::now()` (wall-clock 2026-05-18) → TF buffer rejects every transform as outside the 10s extrapolation window. With `--clock`, ros2 nodes wait for `/clock` and operate in bag-time → TF buffer happy.
5. **Multi-pass preflight kill** (3 iterations × 1s gap) in `run_jetson_bag_full_load.sh`. Single-pass `pkill -9 -f` sometimes leaves stale `rosbag2_player` processes whose argv took longer than the signal delivery window. Discovered when 2 concurrent bag processes were publishing `/livox/imu` with 132s-offset timestamps causing Point-LIO "imu loop back, clear deque" rejection of every message.
6. **Point-LIO Odometry topic remap**: Point-LIO publishes Odometry on `/aft_mapped_to_init` (HKU upstream name) NOT `/Odometry` (fast_lio convention). dfloreaa's port kept the upstream name. Subscribers (`fast_lio_tf_adapter`, CFPA2, Nav2) expect `/robot/Odometry` so the launch remap is `("/aft_mapped_to_init", f"/{ROBOT_NS}/Odometry")`. Without this, `/robot/Odometry` has 0 publishers, fast_lio_tf_adapter never gets odom, CFPA2 has no robot pose, Nav2 stuck in "Robot is out of bounds of the costmap".
7. **Staggered launch startup** (`TimerAction` with 15/18/22/28/30s delays) for the heavy subscribers (elevation_mapping at +15s, grid_map_to_occupancy at +18s, Nav2 at +22s, CFPA2 at +28s, bridge at +30s). Point-LIO + 3 statics + adapter come up at t=0; bag at t=12. This is a workaround for the Point-LIO SIGSEGV race — see #1 in TL;DR. With staggered startup the survival rate rose from ~50% to maybe ~70%; with `respawn=True respawn_delay=2.0` added on top, every launch eventually succeeds within 1-2 respawns. Doesn't help with the SLAM divergence (#2) though.
8. **`grid_map_to_occupancy_grid.py` rclpy/rospy idiom fix**: replaced 3× `self.get_logger().warn_throttle(clock, ms_int, msg)` (rospy idiom — `RcutilsLogger` has no `warn_throttle`) with `self.get_logger().warn(msg, throttle_duration_sec=N)` (rclpy idiom). Without this fix the node crashed on first throttled-warning path: `AttributeError: 'RcutilsLogger' object has no attribute 'warn_throttle'`.

### Why bench (weaker Orin Nano 8GB) was faster than onboard (stronger Orin NX 16GB)

Initial measurement showed bench `/robot/Odometry` at 10 Hz vs onboard record of 5.7 Hz average. User pushed back: "弱机器不可能比强机器快 1.75×". The challenge prompted concrete experiments:

| Hypothesis for onboard 5.7 Hz | Action | Verdict |
|---|---|---|
| Bag was being throttled by fast_lio backpressure and `ros2 topic hz` is fooled by wall-clock measurements | Direct RTF measurement: sample `/clock` at two wall-clock points 20s apart. wall_dt=21.47s, sim_dt=22.98s → **RTF = 1.07** (within ±5% noise of 1.0). | **REFUTED.** Bag really is wall-clock real-time. |
| `rosbag record` running concurrently with FAST-LIO stole CPU | Bench experiment: keep fast_lio + full stack running, ADD `ros2 bag record` of 8 heavy topics (cloud_registered_body + elevation_map + all of /livox/* + tf) simultaneously. Disk write 38 MB/s sustained. | **REFUTED.** fast_lio CPU was 5.5% before, 5.3% after. Odometry rate went from 10.004 Hz to 10.329 Hz (UP). System loadavg 6.4/6 cores under combined load. |
| **`ros1_bridge` serialization between Noetic FAST-LIO and ROS 2 record path** | Inferred from bag filename `onboard_noetic_*_ros2_raw` (records via bridge). PointCloud2 ~100 KB × 10 Hz per topic crossing bridge requires full re-serialize. Single-core CPU cost is substantial. | **LIKELY culprit.** Not directly tested but the only remaining strong candidate. |
| Onboard `pcd_save_en` accidentally `true` | Default in fast_lio's mid360.yaml is `false`. We explicitly force `false` in the bench launch. Onboard launch may have set `true`. | Possible — would explain unbounded memory + degradation. Not verified. |
| Thermal throttle on real Go2 (sealed belly, sun, walking-induced friction heating) | Bench thermal snapshot under full load: CPU 54.1°C, GPU 53.7°C, SoC 52.8°C. **Bench has 30°C headroom to throttle (~85°C)**. Real Go2 belly is much hotter — even quiescent it would run 70°C+. | Plausible additional factor. Not measurable from this bench. |
| **Concurrent onboard workload (Mid-360 driver IO, motor controller, camera streams, CHAMP, etc.)** | Bench has only the SLAM+autonomy stack; real onboard has 20+ extra real-time threads competing for the same 8 cores. | Almost certainly a contributing factor. |

**Net conclusion**: fast_lio is *not algorithmically slow* on ARM. Onboard 5.7 Hz was a record/integration artifact, not a SLAM algorithm limit. The 2026-05-13 CLAUDE.md note ("FAST-LIO2 ikd-tree grew unbounded → 9→4.4 Hz") was likely the *pcd_save_en=true* configuration, not the algorithm itself.

### Files shipped

- `src/vendor/livox_ros_driver2_msgs/{CMakeLists.txt, package.xml, msg/{CustomMsg.msg, CustomPoint.msg}}` — msgs-only stub package
- `src/vendor/point_lio_ros2/` — uncommented CustomMsg path (preprocess.h, preprocess.cpp, laserMapping.cpp), added `find_package(livox_ros_driver2 REQUIRED)` + `ament_target_dependencies(... livox_ros_driver2)`, plus `<depend>livox_ros_driver2</depend>` in package.xml. Build verified on desktop + Jetson. (Kept in tree even though unused — useful reference for any future port debugging.)
- `src/vendor/point_lio_ros2/config/mid360_go2_real.yaml` — Mid-360-on-walking-Go2 tuned config (`start_in_aggressive_motion: true`, `satu_acc: 5.0`, `publish.path_en: false`). Not on the production path now but preserved.
- `scripts/real/orin_nano_bag_full_load.launch.py` — full bag-replay HIL launch: fast_lio + 3 statics + `fast_lio_tf_adapter` + elevation_mapping_cupy + grid_map_to_occupancy + Nav2 stack + cfpa2_single_robot + cfpa2_to_nav2_bridge. With staggered TimerActions and respawn options.
- `scripts/real/run_jetson_bag_full_load.sh` — runner: multi-pass preflight kill (incl. `rosbag2`, `yes`, `point_lio`, etc.), DDS shm cleanup, `ROS_DOMAIN_ID=42` isolation, `ELEVATION_MAPPING_FORCE_CUPY=1` env. Prints the bag-play one-liner the user runs in a second window.
- `scripts/real/deploy_to_orin_nano.sh` — updated vendor list: `point_lio_ros2` + `livox_ros_driver2_msgs` (replaces old `livox_ros_driver2` + `Livox-SDK2`). Build target now includes `point_lio`.
- `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py` — 3× rospy→rclpy `warn_throttle` → `warn(throttle_duration_sec=)` migrations.

### Open / next

- **CFPA2 + grid_map_to_occupancy are the next optimization targets.** Single-core Python on ARM is fundamentally slow at the ~5 Hz target rates. Options: numba JIT the BFS/clustering hot paths in CFPA2, or rewrite grid_map_to_occupancy in C++ (it's a fixed-grid OccupancyGrid converter with seed-flood + threshold logic — straightforward to port).
- **Point-LIO ROS 2 port C++ fix** is non-trivial (DDS publisher init race + SLAM divergence on Mid-360 walking data) — not worth the bench effort; production stays on fast_lio. Keep the vendored copy + uncommented CustomMsg path for future reference if a different port appears upstream.
- **2026-05-13 CLAUDE.md note about fast_lio degradation** should be updated to reflect this bench test result — with `pcd_save_en=false` + `extrinsic_est_en=false` (current yaml defaults), 170s on the same ops2 bag shows no degradation. The earlier observation likely had different config.
- **5+ minute long-run test** (loop the 180s bag 2-3 times) needed to definitively rule out long-tail ikd-tree growth on bench. Started but not completed in this session.

## Active state (2026-05-18 evening) — bridge-as-obstacle root-cause hunt: it's the ingest, not the CNN

Follow-up to the morning's trav-CNN fine-tune commit (2dd8664). The exploration sim worked end-to-end and the robot spawned correctly at (0,0), but **RViz showed overhead bridges / awnings as red lethal cells** in `trav_fused`, even with the fine-tuned weights. Spent the afternoon ruling out hypotheses; final fix was upstream of the CNN.

### What was tried and what it told us

| Hypothesis | Action | Verdict |
|---|---|---|
| CNN under-trained on bridges → train more epochs | Inspected curve: val_mse plateau at ep100 (0.04). Adding epochs would not move it. | **Not it.** |
| CNN sees too narrow a context (7×7 at 0.10m = 0.7m FoV, smaller than typical bridge → patch looks like a wall step at the bridge boundary). | Added `--patch-stride` to [`build_trav_dataset.py`](scripts/training/build_trav_dataset.py): same 7×7 cells but spaced 0.20m, total FoV 1.4m. Trained `weights_ops2_wide.dat` (val_mse 0.088, val_acc 0.904 — worse than tiled 0.040/0.948). | **Not it.** Wider FoV degraded the model. The 7×7 base CNN already had enough info; smearing it over 1.4m hurt fine-detail walls. |
| CNN really is wrong on uniform high-z patches | Synthetic eval: fed CNN three patch types — floor (z=0), bridge top (uniform z=3m), wall edge (half z=0 / half z=3m). **All weights (pretrain / ops2_tiled / ops2_wide) predict 0.84+ free for both floor AND bridge, and 0.000 lethal for wall edge.** The CNN was correctly classifying bridges as free the whole time. | **CNN ≠ root cause. Stop blaming the model.** |
| `grid_map_to_occupancy_grid.elevation_cost_enabled=True` was unconditionally adding 90 cost for cells with z > 1.5m, overriding CNN's free verdict | Disabled it via [nav_test_3d_explore.launch.py:292](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py#L292). Bridges still lethal. | **Not it** (but `elevation_cost_enabled=False` left as default since it didn't help and added unnecessary penalty). |
| Nav2 global_costmap.static_layer reads `/robot/map` (octomap 2D projection — octomap traces LiDAR hits regardless of z, so bridge tops project DOWN to occupied) instead of `/robot/traversability_grid` (CNN-fused, has bridge override) | Switched static_layer `map_topic` from `/robot_b/map` to `/robot_b/traversability_grid` in [nav2_go2_full_stack.yaml](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml). Bridges still lethal. | **Not it.** Nav2 was already going to do the right thing once trav_grid is right — but trav_grid was wrong upstream. |
| **elevation_mapping_cupy itself ingests bridge-top points and writes z=3-4m to the cells beneath them. CNN then sees those cells as either uniform-high (correctly classified free) or as edges (incorrectly classified lethal due to half-bridge half-no-data patches at bridge boundaries).** Simplest fix: cap `max_height_range` so bridge points never enter the map. | [elevation_mapping.yaml](src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml): `max_height_range: 5.0 → 1.7` (sensor sits at z≈0.3m, 1.7m above sensor = z≈2.0m world). Also lowered `ramped_height_range_b: 2.5 → 1.7` for the slant-range gate. | **✓ FIX. Verified in sim.** Bridges cleared in `trav_fused`. |

### Conclusion

The CNN was correctly classifying every patch type it could possibly see. The bug was that **bridges were entering the elevation map at all**, polluting the input regardless of how good the model got. The fix sits one level upstream from where we were looking, and is a one-line YAML change — but it took eliminating four other plausible suspects (CNN training, patch size, downstream elevation_cost, Nav2 static_layer source) to know that.

A second secondary fix shipped along the way:
- **Nav2 global_costmap static_layer** now reads `/robot_b/traversability_grid` instead of `/robot_b/map`. The cnn-fused trav layer with bridge override is strictly more correct than octomap's 2D z-collapsed projection for planning, even if the bridge issue happens to be solved upstream now.

### Other touch-ups this afternoon

- **`scripts/real/flatten_floor_height.py`** removed by user during a roll-back iteration (not on the current path anyway; we ended up with the cleaner `strip_floor_with_patchwork.py → ransac-tiled` mode for any future cleanup needs, and `scans_v4_sparse.obj` as the actual sim asset).
- **`build_trav_dataset.py --patch-stride N`** added during the hypothesis hunt and left dormant. Use the default (stride 1, 7×7 at 0.10 m = 0.70 m FoV) for production fine-tunes — the wide-context variant degraded val_mse without solving the bridge issue. CNN architecture would also have to scale dilations via `traversability_filter.py` to deploy a stride>1 model, which we did not do.
- **Simple 3D ramp demo** runs unchanged via `./scripts/launch/nav_test_3d_explore.sh` (demo_ramp.xml default) or `nav_test_3d_explore_go2.sh` (Menagerie Go2 body). With today's `max_height_range: 1.7` change, demo_ramp is unaffected (its tallest features are well under 2m) but ops2 bridges should clear.

## Active state (2026-05-18) — ops2 trav-CNN fine-tune end-to-end + spawn bug + Nav2 tightening

End-to-end: built the offline label-generation → 7×7 patch extraction → CNN fine-tune → sim-test pipeline for elevation_mapping_cupy's 120-param traversability filter. The sim now spawns at (0,0), uses fine-tuned weights by default, drives autonomous exploration across the ops2 scene, and goes ~15m down the main corridor in 90s.

### The shipped pieces

1. **Heightmap → labels → patches → CNN fine-tune pipeline** in [`scripts/training/`](scripts/training/):
   - [`build_float_heightmap.py`](scripts/training/build_float_heightmap.py) — top-down z(x,y) from mesh, float32 npz
   - [`auto_label_heightmap.py`](scripts/training/auto_label_heightmap.py) — slope+step+height-above-floor rules on real ops2 mesh (vs `synth_terrain_dataset.py` which uses synthetic toy ramps)
   - [`polish_trav_labels.py`](scripts/training/polish_trav_labels.py) — morphological speckle remove + bridge override (height > 1.2m AND slope < 20° → free)
   - [`build_trav_dataset.py`](scripts/training/build_trav_dataset.py) — 7×7 patch extraction with rich augmentation (rot×4, flip-LR/UD, gaussian noise ±0.03m × N rounds, dropout, tilt ±5°, gaussian blur). 6M patches from 60k labelled cells.
   - [`train_trav_filter.py`](scripts/training/train_trav_filter.py) — already existed; extended with **weighted MSE / BCE / focal** loss (`--lethal-weight 3.0` for FP-lethal-over-FN-lethal bias), **label smoothing 0.05** (guards against ~10% label noise), **pretrain mix 30%** every epoch (anti-catastrophic-forgetting from synth `weights_pretrain.dat`), **weight_decay 1e-4**, **early stopping**, and **periodic checkpoints** every N epochs.
   - Result: val_mse 0.04, val_acc 0.95 across all fine-tuned variants (v2 / snap / flat / ransac / tiled). Pretrain baseline was 0.117 / 0.86. Per-class on tiled dataset: trav_pred 0.92, lethal_pred 0.11. **5090: 6M patches × 150 epochs in ~30 s** (120-param network, batch 4096, GPU 50% util — kernel launch overhead, not compute-bound).

2. **5 mesh-cleanup variants explored** for the visual mesh feeding sim LiDAR. All trained equally well (CNN sees similar patch distributions), differ in **sim realism**:
   - **strip** (patchwork delete-tris) — has holes, LiDAR escapes
   - **snap** (patchwork + project ground to z=0) — flat but loses ramp tilt
   - **flat** (no patchwork, z<0.10m → 0) — simpler, catches 88% more low-z noise than patchwork (284k vs 241k verts)
   - **ransac** (patchwork + single-plane RANSAC) — preserves 0.43° real building tilt; ground std 16cm = expected tilt × 80m extent
   - **tiled** (patchwork + per-2m-tile RANSAC) — preserves multi-level floor + ramps (p5 z=−0.15m → p95 z=+0.19m, true 0.34m height range across the building). Spawn tile has std 4mm (essentially flat).
   - User reverted aggressive variants (orphan-strip dropped 79k debris tris, vertex-cluster 0.4m got us to 52k tri) because they cut too much; we ended back at original mesh (`scans_v4_sparse.obj`, 56k v / 150k tri) with `pos="0 0 0"` so the mesh's natural ground at z=−0.28m sits one Go2 height (~30cm) below the MJCF floor plane (z=0) acting as visual backdrop.

3. **THE spawn-position bug** — robot was spawning at **(25.x, -15.x)** instead of (0,0) despite keyframe qpos="0 0 0.32". Root cause at [`slam_ops2_v4_go2_real.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_real.xml#L1213): `<body name="base_link" pos="29.51 -15.77 0.32" ...>` — the body's default `pos` attribute was hardcoded to a stale old spawn from a different scene. With a `<freejoint>`, both the body `pos` and the keyframe qpos contribute; the body pos was apparently winning during sim startup. Fixed in both MJCF variants. Verified via headless MuJoCo: `qpos[:3]=(0, 0, 0.32)`. Live sim confirmed: GT pose at t=0 was (-0.03, +0.0, +0.28), exploration drove the robot to (15.39, -9.14, 0.28) over 90s.

4. **Nav2 Go2 collision tightened** ([`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml)) to fit narrower corridors: footprint 0.70×0.40m → **0.64×0.36m**, MPPI `collision_margin_distance` 0.10 → **0.05m**, local `inflation_radius` 0.30 → **0.22m**, global 0.25 → **0.20m**. Effective rejection envelope: 0.90×0.60m → **0.74×0.46m** (-16cm wide, -14cm tall).

5. **Default trav weights = fine-tuned**. [`nav_test_slam_ops2_v4_go2.sh`](scripts/launch/nav_test_slam_ops2_v4_go2.sh) now auto-loads `weights_ops2_tiled.dat` when present; falls back to `weights_pretrain.dat` only if no fine-tune available. Boot log explicitly prints which one is in use.

6. **`gt_passthrough` mode for `fast_lio_tf_adapter`** ([`scripts/runtime/fast_lio_tf_adapter.py`](scripts/runtime/fast_lio_tf_adapter.py)) — env `GT_PASSTHROUGH=1` makes the adapter subscribe to `/<ns>/odom/ground_truth` and emit TF directly from GT, bypassing Fast-LIO. Useful for isolating spawn-position issues from SLAM bootstrap timing.

7. **`elevation_cost_max_h: 1.00 → 1.5m`** ([`nav_test_3d_explore.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py#L295)) — broadens the height band over which `grid_map_to_occupancy_grid` scales elevation cost into the 0–90 occupancy range.

### Other detours (kept as scripts but not on the main path)

- **Sonata (Meta self-supervised PTv3) semantic segmentation** [`scripts/real/sonata_inference.py`](scripts/real/sonata_inference.py), [`sonata_to_instances.py`](scripts/real/sonata_to_instances.py), [`sonata_visualize_instances.py`](scripts/real/sonata_visualize_instances.py). Got it running on Blackwell sm_120 by upgrading `spconv-cu120` → `spconv-cu126 2.3.8` + `cumm-cu126` (the prior version had no sm_120 cubins and silently SubMConv3d-segfaulted). Achieved 0.954 acc per-pixel on the v4 mesh. **But the ScanNet-20 taxonomy is wrong for our outdoor/semi-outdoor building scene** — patchwork picked 26.1% of verts as ground (matching truth) but Sonata mis-labelled most building structures as "table" or "bed". Dropped semantic path. Sonata's 5 instances (with z<1.5m guard) were still extracted into 120 CoACD convex-decomposition collision geoms (`ops2_inst_*` in MJCF) — kept for collision-only use.
- **Patchwork++ ground stripping** [`scripts/real/strip_floor_with_patchwork.py`](scripts/real/strip_floor_with_patchwork.py) (recreated inline after user deleted it during one iteration). Multiple modes: `strip` (delete tris, leaves holes), `snap` (project ground to z=0), `ransac` (single plane), **`ransac-tiled`** (per-tile plane, preserves multi-level floors). With z<0.05m guard, rescued 14k pillar-base verts that patchwork over-classified as ground.
- **Mesh editor GUIs** (none turned out to be the right UX): [`mesh_box_carver.py`](scripts/real/mesh_box_carver.py) 3D AABB select-and-delete, [`mesh_height_cutoff.py`](scripts/real/mesh_height_cutoff.py) 2D top-down + max-z cutoff slider, [`trav_labeler.py`](scripts/training/trav_labeler.py) 2D paint w/ z-band slider + Bresenham straight-line tool, [`trav_labeler_3d.py`](scripts/training/trav_labeler_3d.py) Open3D shift-click point picker, [`trav_threshold_tuner.py`](scripts/training/trav_threshold_tuner.py) live-slider rule tuner. User preferred automated path over manual labeling.
- **`flatten_floor_height.py`** [`scripts/real/flatten_floor_height.py`](scripts/real/flatten_floor_height.py) — height-only snap (no patchwork): every vert with z<0.10m → z=0. Catches 88% more low-z noise than patchwork but loses any real curb/step <10cm.

### Open problem — overhead bridges treated as lethal at runtime

The trav grid (visible in RViz `/robot/traversability_grid`) marks overhead bridges/awnings as **red lethal**, NOT free as we want for "robot walks under". Why: at offline label time, `polish_trav_labels.py` applies a `bridge_height_m=1.2m AND slope<20°` rule that flips high+flat cells to FREE. The CNN learns this from labels. **At runtime**, the elevation_mapping_cupy CNN sees a 7×7 patch from the live heightmap (max-z per cell from LiDAR) — but there is no equivalent height-based override in the *output* pipeline. The CNN's per-patch output gets multiplied with the `ramp_safe` analytical fallback ([grid_map_filters.yaml](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml)) which is meant for ramp recovery, not bridge clearance.

Two ways to fix:
1. Add a post-CNN filter in `grid_map_filters.yaml`: `height_above_floor > 1.5m AND slope < 20° → trav_fused := max(trav_fused, 0.95)`. Same logic as offline polish, applied to the live grid before publish.
2. Train the CNN with patches that include FLOOR underneath a bridge (i.e. patches that show "z=4m here, but z=0 just 1m laterally" → label=trav). Currently our patches show only the top-down max-z, so a bridge patch looks like a tall obstacle.

(1) is the quick fix; (2) is the principled fix but needs a different patch builder.

## Active state (2026-05-16) — real-world walk → MuJoCo collidable scene (Go2 explores SLAM mesh autonomously)

End-to-end: offline replay of a real-robot bag → static-only mesh → Go2 spawned and trotting inside it under CFPA2 + Nav2, with only foot-floor contacts.

**Pipeline** (all scripts in [`scripts/real/erasor/`](scripts/real/erasor/)):
1. **`offline_slam.sh tag=ops2`** — Fast-LIO 2 on raw `/livox/lidar` + `/livox/imu` from a Noetic bag. No loop closure: SC-A-LOAM's ScanContext gets fooled by glass-window reflections into false LC, which warped the map worse than drift. Verified by building the full SC-A-LOAM + ERASOR Docker stack ([`scripts/real/erasor/Dockerfile`](scripts/real/erasor/Dockerfile)) and seeing LC-corrected output corrupted. Output: 25.8M points → [`bags/meshes/ops2_final/scans.obj`](bags/meshes/ops2_final/scans.obj).
2. **`density_filter.py --min-pts 8`** — temporal-consistency filter via points-per-voxel count. Static surfaces accumulate 60-700+ points/voxel across all keyframe contributions; pedestrian trails + glass-reflection outliers only 1-5 points/voxel. Threshold at 8 keeps 70.3%, removes the rest. Decoupled from mesh reconstruction (the user's key insight) — Poisson on already-clean cloud doesn't hallucinate over trail-shaped gaps.
3. **`pcd_to_mesh.py --method poisson --voxel 0.02 --depth 11 --density-pct 10`** + quadric decimation to 500K triangles + `cluster_connected_triangles` cull (keep ≥ 2000 tri components) → 240k v / 473k f.
4. **RANSAC ground alignment** in bottom-5%-z subset (initial bottom-30% picked up walls and gave 2.5° residual tilt; tight bottom-5% gives **0.19°**). Mesh ground forced to z=0 ±0.04m.

**5 MuJoCo integration fixes** (each was a blocker for autonomy):
1. **Whole-mesh convex hull pushed robot through floor.** MuJoCo collides `<geom type="mesh">` via its **convex hull** by default. An 80×32×10 m scene-wide hull pushed the body box to z=-0.078 (8cm below floor) regardless of spawn z. Fix: [`tile_mesh.py`](scripts/real/erasor/tile_mesh.py) splits mesh into 8×4 XY grid (22 non-empty tiles, max 42K tri/tile). Each tile's hull approximates only its local geometry → robot stands at proper z=0.236 m.
2. **CHAMP can't stand a robot whose initial joints are at 0** (legs straight down). With nominal joints `(hip=0, thigh=0, calf=0)` and body spawned at z=0.60, legs hit floor extended; CHAMP commands the fold-to-standing pose but motors can't lift body weight against tucked-leg geometry. Fix: `<keyframe>` block with `qpos` initialized to `(thigh=0.9, calf=-1.8)` per leg — robot spawns already in trotting-ready pose at z=0.32. nq=**19** for Go2 (no foot joints in MuJoCo despite ROS-side `*_foot_joint` names from URDF parsing).
3. **`initial_pose_guard.py` re-pins robot to (0, 0, 0.38) for 14 s after spawn** via `/gazebo/set_entity_state` calls. In MuJoCo mode that service doesn't exist so the calls no-op, BUT the script still consumes the `spawn_x/spawn_y/spawn_z` launch args; passing them via the wrapper overrides the (0, 0, 0.38) default at the param-store level (no functional override needed for MuJoCo, but matters for future Gazebo runs).
4. **Wrong launcher** — `nav_test_mujoco_fastlio.launch.py` doesn't include the elevation_mapping + filter_chain + grid_map_to_occupancy_grid trio, so CFPA2 spins for hours warning `Waiting for map topic from: robot`. Only [`nav_test_3d_explore.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py) has them. New wrapper [`scripts/launch/nav_test_slam_ops2_go2.sh`](scripts/launch/nav_test_slam_ops2_go2.sh) calls 3d_explore.
5. **Spawn point selection** — the bag's first Fast-LIO odom is at (0, 0, 0) in `camera_init`, after alignment that's a poor xy spot (mesh has walls/clutter within 2 m). Sweep over the mesh bbox finds (5.12, -8.76) as best: 148 ground verts in 1 m radius, **0 mesh verts inside robot body volume** within 0.8 m. Encoded as the `<keyframe>` qpos x/y.

**Verified end state** ([`slam_ops2_go2.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_go2.xml) + [`scripts/launch/nav_test_slam_ops2_go2.sh`](scripts/launch/nav_test_slam_ops2_go2.sh)):
- Standing: base_link z = **0.236-0.241 m**.
- Contacts: only `floor|*_foot_collision` pairs (no body collision).
- Trotting: foot contact state `[F, F, F, T]` cycling — 1-2 feet on ground at a time.
- Motion: position trace **(4.68, -9.02) → (3.07, -9.70) → (4.02, -10.02)** in 12 s — actively exploring.
- Stack: trav_grid 1.7 Hz, cmd_vel 20 Hz, way_point_coord driving to (17.55, -13.45).

## Active state (2026-05-15) — ETH elevation mapping + CNN traversability + ramp_safe fusion live; Go2W still tips at ramp edges (open)

Single-session end-to-end stand-up of the ETH RSL elevation_mapping_cupy stack with the CNN traversability filter actually running, fused with the analytical chain, and consumed by Nav2 in 2D. Robot autonomously explores demo_ramp via CFPA2 → Nav2 with no scripted ramp helper. One remaining open issue: Go2W tips when the planner routes it near the ramp foot transition / platform cliff edge — see [docs/claude/ramp_tipover_open_problem.md](docs/claude/ramp_tipover_open_problem.md).

### The win

1. **Pure-cupy ETH CNN backend on Blackwell sm_120.** torch 2.7.1 ships sm_50..sm_90 cubins only — `torch.cat` fails with `CUDA_ERROR_NO_BINARY_FOR_GPU` on the 5090, which is why a prior Phase-3.3 patch had *disabled* the CNN at [`elevation_mapping.py:408-417`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py#L408-L417) and left layer 3 at its init value 1.0 (so `traversability` was a no-op). Added [`get_filter_cupy()`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/traversability_filter.py) — 65 lines of pure cupy reimplementing the 3-branch dilated 3×3 conv stack + 1×1 fusion + `exp(-|·|)`. Float32 match vs numpy reference < 1e-7. Runtime backend selector at [`elevation_mapping.py:145-163`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py#L145-L163) picks the cupy filter when `cudaDeviceProperties.major >= 10`. CNN populates the `traversability` layer at ~5 Hz; on demo_ramp it correctly marks wall TOPS lethal (not just the 1-cell rim), catching ~450 more wall cells than the analytical chain.
2. **CNN ↔ analytical fusion in the filter chain** ([`grid_map_filters.yaml`](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml)). CNN over-rejects ramps (training was on mostly flat ground). Built a soft analytical rescue: `slope_margin = clamp((0.5236−slope)/0.5236, 0, 1)`, `step_margin = clamp((0.06−step_residual)/0.06, 0, 1)`, `ramp_safe = clamp(slope_margin·step_margin·100, 0, 1)` (gain 100 + clamp turns the product into a near-binary mask). Then `trav_fused = max(traversability, ramp_safe) = 0.5·(a + b + |a−b|)` (EigenLab has no element-wise max). Live numbers: ramp cells CNN_lethal 91 → fused_lethal 9 (out of 1012 eligible); wall cells preserved (1834 → 1834); flat ground cleaned (452 → 0). Nav2 reads `trav_fused` from `/robot/traversability_grid`; raw CNN and analytical `trav_eth` remain as parallel comparison layers in RViz.
3. **2D CFPA2 on the fused trav grid.** With `trav_fused` producing a clean 2D OccupancyGrid, CFPA2's BFS-on-OccupancyGrid (`ig_dimension: 2d`, `planning_map_topic_suffix: /traversability_grid`) is sufficient — no need for the 3D voxel-cluster IG path. Removed `ramp_ascent_goal_node`, `ramp_cmd_vel_assist_node`, and `frontier_3d_test_node` from the launch. Pure CFPA2 → Nav2 autonomy.
4. **Height-based extra cost.** Added optional `elevation_cost_enabled` to [`grid_map_to_occupancy_grid.py`](src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py): cell cost mixed via max() with `clamp((elev − h_min)/(h_max − h_min) · v_max, 0, v_max)`. With `h_min=0.05, h_max=1.00, v_max=90`, live values on demo_ramp: flat ground mean cost 0.0, ramp_foot 0.2, ramp_mid 34, ramp_top 82. Planner now prefers flat-ground routes when reaching the same frontier doesn't require climbing.
5. **Mid-360 sim densification + real-robot mount calibration.** Replaced the uniform-grid raycast in `mujoco_ros2_control`'s lidar plugin with Livox-SDK official Risley non-repetitive scan-pattern replay (`scan_patterns/mid360.csv`, 800k samples = 4 s, 20k rays/frame + σ=2 cm Gaussian range noise). Calibrated Mid-360 mount (roll=−2.11°, pitch=+15.10°) in every sim TF source — 18 MJCFs, 2 xacros, the `go2w.urdf` snapshot, `pointlio_gazebo*.yaml` extrinsics, gbplanner3 sim static TF publisher, and the gbplanner3 demo3 launcher. Sim point cloud now matches real-robot density and the elevation map renders without the spurious 2° tilt that the previous 13°-pitch-only guess introduced.

### Debugging journey (this session)

The CNN re-enable + fusion + autonomy hookup happened in a tight feedback loop where each fix exposed the next layer's problem:

1. **CNN was off.** First reading of `elevation_mapping.py:408-417` revealed the Phase-3.3 commit had *deliberately commented out* the filter call. `use_chainer: false` was misleading — switching it to `true` wouldn't help because `chainer` isn't even installed. Wrote the cupy backend instead.
2. **RViz default display was reading the wrong layer.** Initial GridMap display had `Color Layer: traversability` but elevation_mapping_cupy's `traversability` layer was all-NaN until the CNN re-enable landed. The actually-computed analytical chain was `trav_eth`. After re-enable both layers had real data — RViz now shows `trav_fused` by default with raw CNN and `trav_eth` as togglable comparisons.
3. **`step_cost` topology bug (analytical chain).** For a wall TOP cell, slope ≈ 0, step_height in 3×3 window ≈ 0 (window all on wall top), so step_cost = 0 → wall top renders as FREE with only a 1-cell lethal rim. Added a max-filter dilation (`maxOfFinites(step_cost)` in 7-cell window) plus a separate `wall_seed = step_height − 0.25` gated detector that decouples wall-vs-ramp signal. Walls now fully red across thickness; ramps untouched. This was needed before CNN came online because CNN itself catches the same case — but the analytical chain is the fallback / comparison layer, so it's worth fixing.
4. **CNN fixes walls but rejects ramps.** First fused-layer screenshot: walls solid red ✓, ramp glowed red too. The CNN was trained on mostly-flat data → sustained slope reads as step-like. Rather than retrain (multi-day infra build), added the `ramp_safe` analytical rescue that `max()`-fuses on top of CNN.
5. **`ramp_safe` initially under-shot.** First version: `ramp_safe = slope_margin · step_margin`, no gain. On a 14° ramp `slope_margin ≈ 0.53` → ramp_safe ≈ 0.53 → trav_fused ≈ 0.53 → yellow in RViz. Added `· 100` then clamp(0, 1) to saturate to near-binary {0, 1} mask.
6. **`filters::FilterChain` silently dropped filters.** First fusion-yaml had filters named `filter9b, filter9c, filter9d, ...` and `filter23a, filter23b, ...`. The chain loaded only `filter1..filterN` contiguous; everything past the first non-integer suffix was silently ignored — observed in the log as `out layers=19` (missing `ramp_safe`, `trav_fused`) without any FATAL. Renumbered all filters to contiguous integers. Memory note: [feedback_filter_chain_integer_keys.md](docs/claude/memory/feedback_filter_chain_integer_keys.md).
7. **EigenLab has no `>` operator.** First attempt at the indicator mask used `(ramp_safe > 0.0) .* 1.0`; loaded as `Unknown variable 'ramp_safe > 0.0'.` and crashed the filter_chain_runner. Replaced with clamped-margin product + gain.
8. **Robot stuck at `no_frontiers` despite obvious unknown territory.** With everything working, CFPA2 reported `no_frontiers` and the robot sat at (1.4, 0.75). Live BFS analysis: 1690 raw frontier cells reachable from the robot's position. The status came from CFPA2's *post-extract* filters:
   - `cfpa2_frontier_obstacle_clearance_m: 0.35` (4-cell radius from any lethal) — the 2 m ramp corridor has walls at y=±1 m → centroid is always within 0.35 m of a side wall
   - `cfpa2_frontier_min_unknown_cells: 20` over `cfpa2_frontier_unknown_check_radius_m: 0.40` (4-cell radius) — the unknown-check window picks up corridor walls so the "live unknown" count collapses to ~0
   - **`cfpa2_max_goal_distance_m: 2.5`** — the load-bearing filter. After the robot maps everything within 2.5 m to free, the next reachable frontier ring is 3–6 m out. `_goal_too_far` drops all of them. Targets list becomes empty → `no_frontiers` reported (not `no_reachable`, because the empty check fires before utility evaluation).
9. **Scene-specific overlay rather than global relax.** Rather than touching the base `cfpa2_single_robot.yaml` (open-scene safe defaults), added a `cfpa2_config_overlay` launch arg to `nav_test_mujoco_fastlio.launch.py` and a [`cfpa2_single_robot_demo_ramp.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot_demo_ramp.yaml) overlay: clearance 0.20, min_unknown 5, unknown_check_radius 0.30, max_goal_distance 15.0. Verified loaded via `ros2 param get`.
10. **Threshold/cost tuning to stop Go2W tipping (partial — see open problem).** Iterated free/lethal: 0.30/0.15 → 0.22/0.10 (too permissive, robot grazed ramp foot) → 0.55/0.25 → 0.60/0.30 (final). Added `elevation_cost_enabled` height penalty (h_min=0.05, h_max=1.00, v_max=90). Bumped Nav2 InflationLayer to `inflation_radius: 0.60, cost_scaling_factor: 2.5`. Robot still tips occasionally — the ramp foot is *legitimately* traversable per every sensor; tipping is a dynamic-stability failure our 2.5D cost layer can't observe. Open problem document lists 5 candidate next steps from "tighten ramp_safe envelope" to "train CNN on tilted terrain".



The ad-hoc 6-step 2D traversability projection in `nvblox_frontend/mapper_node.cpp` has been replaced with the full ETH RSL pipeline. All 7 phases of [plans/2026-05-14-trav-grid-rewrite.md](docs/claude/plans/2026-05-14-trav-grid-rewrite.md) are committed. Phase 7 (A/B validation run) is the remaining step.

**Pipeline (activated when `nav_costmap_mode:=3d`, the default):**
1. `elevation_mapping_cupy` → `/robot/elevation_map_raw` (3 layers: elevation, variance, traversability)
2. `filter_chain_runner` (trav_cost_filters) → `/robot/elevation_map_filtered` (12 layers: adds normal_vectors_{x,y,z}, slope, slope_cost, roughness, roughness_cost, step_height, step_cost, traversability overwritten)
3. `grid_map_to_occupancy_grid` (trav_cost_filters) → `/robot/traversability_grid` (OccupancyGrid, Nav2 cost convention: 0=free if trav≥0.7, 100=lethal if trav<0.3)

**Key bugs fixed during integration (all in one session):**
- `/**:` wildcard required in YAML (not bare `filter_chain_runner:`) so params load under `/robot` namespace
- `ThresholdFilter` expects `layer:` not `condition_layer:` + `output_layer:`
- EigenLab `max(scalar, matrix)` invalid → use multiplicative traversability: `(1-slope_cost)*(1-roughness_cost)*(1-step_cost)`
- `ament_cmake_python` installed egg-info lacks `entry_points.txt` → Python executable installed via `install(FILES ... RENAME ...)` with execute permissions instead of `console_scripts` stub

**Smoke test verified:** 12 layers in `elevation_map_filtered` at 5 Hz, OccupancyGrid at ~9 Hz, against live demo_ramp sim.

## Active state (2026-05-14) — 3D frontier exploration unstuck: 6 compounding bugs

Day-long debug pass on `nav_test_3d_explore.sh` (demo_ramp + nvblox_frontend + CFPA2 3D IG). Robot kept getting stuck on the same goal forever. Six independent bugs compounded; full details + verification log in [docs/claude/3d_frontier_debugging.md](docs/claude/3d_frontier_debugging.md).

- **Ring-frontier centroid bug (the load-bearing one).** `centroid_world = (xs.mean(), ys.mean(), zs.mean())` of frontier voxels gives a point at the GEOMETRIC CENTRE of the frontier voxel set. For any incremental exploration the frontier voxels form a SHELL surrounding the robot's carved-FREE region → mean lands AT the robot → goal = current pose → no motion, ever. Fix in [`frontier_3d.py:193-225`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py#L193-L225): when `robot_xy` is supplied, pick the frontier voxel with max `‖voxel − robot‖²` instead. Always returns a real boundary point in the direction of largest unexplored extent. Without this, the system literally cannot explore regardless of every other fix below — they are all preliminary cleanup.
- **Why mean-centroid is broken in general.** Mean approximates "where the frontier is" only when the frontier is sharply pointed (e.g. a one-direction hallway). For compact / wraparound FREE regions — the default of every exploration session — it collapses to self-position. Any frontier-based explorer using mean centroid will deadlock the moment the frontier becomes annular.
- **Octomap-style trav_grid projection** ([`mapper_node.cpp`](src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp)). Dropped ~200 lines of polar fan-fill + persistent ray_covered + 1-cell dilation. Now: query nvblox's `occupancy_layer` per column directly. nvblox already does proper 3D Bayesian raycasting; behind-wall voxels stay log_odds=0 → projection gives UNK, no leak. Slope/step filter on highest-occupied-z still gates ramp-vs-cliff (Octomap proper doesn't do that — that's the value-add over plain Octomap).
- **trav_grid is now world-fixed and persistent.** 40 m × 40 m grid, origin locked on first odom, `cls_persist_` member retains FREE/OCC across frames; new-frame UNK does NOT overwrite a prior FREE/OCC. Behind-the-robot map persists, no rolling-window memory loss.
- **3D cluster z-band filter [-0.2, 1.5] m** ([`frontier_3d.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py)). Air voxels above ramp / under ceiling are permanently UNKNOWN (no Mid-360 rays go straight up + return) — they inflated the cluster IG forever. Filtering UNK to robot-actionable z makes volume actually shrink as exploration progresses.
- **Mid-360 geometric blind disk** (3 m). V-FOV starts at -7° → ground first visible at `0.4 / tan(7°) ≈ 3.25 m`. Forced-FREE 3 m disk around robot in `publish_traversability` (preserves OCC verdicts).
- **Dead-frontier filter skipped in 3D mode.** 2D-mode check (require N live UNK neighbours around goal) is incompatible with 3D-mode goals which land in FREE by construction.
- **ClusterTracker** ([`cluster_tracker.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cluster_tracker.py)). Cross-frame cluster identity via world-coord AABB overlap (voxel-index AABBs drift as robot-centric voxels_3d grid origin moves — must convert to world). Tracks per-cluster volume trajectory + attempt count from blacklist events; `non_actionable` flag after N attempts with no shrink → CFPA2 stops chasing dead-end clusters.
- **MuJoCo Mid-360 sim was undersampled.** Hardcoded default `1000 × 20` rays = 2.95° vertical resolution → walls show 1-3 voxels tall in nvblox at 1 m. Bumped via env vars `MUJOCO_LIDAR_HZ/VT_SAMPLES = 1024/96` in [`nav_test_3d_explore.sh`](scripts/launch/nav_test_3d_explore.sh) → cloud points 4880 → 23000 per scan, walls fill all 10 z-layers correctly. The yaml `mujoco_sensor_bridge.yaml::vt_samples` is for a *different* Python node that isn't used by the current C++ plugin; editing it has no effect.

## Active state (2026-05-13) — Point-LIO + gbplanner3 onboard, full stack up

- **Point-LIO replacing FAST-LIO2 as production SLAM** ([docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md)).
  Measured 36% rate degradation on FAST-LIO2 over a 3-min ops2 walk (9 Hz start → 4.4 Hz end while inputs steady at 10/200 Hz lidar/IMU). Root cause: ikd-tree grew unbounded → main-loop iterations stalled, lidar frames dropped in callback queue. Switched to Point-LIO (HKU MaRS 2023, [`src/vendor/point_lio_ros1/`](src/vendor/point_lio_ros1/)): iVox replaces ikd-tree (O(1) avg query), decoupled IMU+LiDAR threads, IMU keeps publishing odom when LiDAR stalls. Verified flat 10.001 Hz `/robot/Odometry` (std 1.1 ms) after `jetson_clocks` + MAXN power mode. Patches mirror our FAST-LIO ROS1 set: livox_ros_driver rename → driver2, advertise paths relative, plus a custom NodeHandle split so `/robot/Odometry` doesn't end up under `/robot/laserMapping/Odometry` (Point-LIO uses `nh("~")` for both params + topics; we add `pub_nh` for publishers).
  **→ Update (2026-05-18 late night HIL bench)**: The 36% degradation claim does *not* reproduce on the bench (Orin Nano 8GB, ROS 2 Humble `fast_lio` = `src/vendor/fast_lio`) with `pcd_save_en: false` + `extrinsic_est_en: false`. 170s of the same `onboard_noetic_20260511_*` ops2 bag → flat 10 Hz, RSS 192 MB constant, RTF=1.07. The original observation was on the **ROS 1 Noetic** `point_lio_ros1` / `FAST_LIO` stack with ros1_bridge in the data path; the bench is native ROS 2 with no bridge hop. The fast_lio degradation may have been a config (pcd_save) or bridge-induced artifact, not an algorithm limit. Also: dfloreaa's **`point_lio_ros2`** ROS 2 port (different package from `point_lio_ros1`) is not usable — see "Active state (2026-05-18 late night)" entry for the SIGSEGV race + SLAM divergence details.
- **gbplanner3 stack built + running on Jetson** (PID 31517 gbplanner_node, PID 31518 pci_general_ros_node, uptime 25 s clean). All 22 UAS vendor repos imported via laptop-side `vcs import` + rsync (Jetson has no GitHub access). 7-step pitfall journey documented in [gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md): src/src/ flattening, `COLCON_IGNORE` blocks catkin_pkg, 5 apt debs scp-installed, gflags ExternalProject pre-stage, `image_transport_plugins` + `manhole_detector_ros` CATKIN_IGNOREs, BT XML path depth (`config/ugv/real/go2/` 4-deep for `../../../bt_xml/` to resolve correctly), v2 → v3 config schema rewrite.
- **gbplanner3 ↔ Point-LIO topic interface**: voxblox baked INSIDE `gbplanner_node` (no separate voxblox_node), subscribes to `/robot/Odometry` + `/robot/cloud_registered_body` natively (no ros1_bridge for SLAM stream — that was the whole point of moving to Noetic). Only `/pci_command_path` (< 1 KB/s) crosses to ROS 2.
- **Updated [scripts/real/gbplanner3/](scripts/real/gbplanner3/)**: `bridge_topics.yaml` trimmed to PoseArray-only (no more 2to1 SLAM topics), `gbplanner_go2.launch` rewritten to match upstream v3 pattern (voxblox-in-gbplanner-node, no robot_config.yaml), `orin_launch_gbplanner.sh` no longer assumes Foxy Fast-LIO; preflights Point-LIO presence.

## Active state (2026-05-11) — Noetic FAST-LIO2 onboard (gbplanner3 prep)

- **Native ROS 1 FAST-LIO2 brought up on the Jetson** in a separate catkin ws (`~/noetic_fastlio_ws/`), parallel to the existing Foxy `~/onboard_ws/`. Eliminates ros1_bridge bandwidth bottleneck for the gbplanner3 voxblox path (heavy `/robot/cloud_registered_body` stream now stays in ROS 1 natively; only the tiny `/pci_command_path` PoseArray still crosses to ROS 2).
- New scripts: [`deploy_noetic_to_jetson.sh`](scripts/real/deploy_noetic_to_jetson.sh) (laptop rsync), [`onboard_fastlio_noetic.sh`](scripts/real/onboard_fastlio_noetic.sh) (Jetson launcher), [`onboard_record_noetic.sh`](scripts/real/onboard_record_noetic.sh) (`rosbag record`, nohup-protected), [`stream_cloud_live.sh`](scripts/real/stream_cloud_live.sh) (Open3D live viewer over ssh binary pipe — replaces X-forwarded RViz which renders black on Jammy + Ogre).
- 11 patches against HKU upstream `FAST_LIO` + `livox_ros_driver2` (livox driver rename, Mid-360s enum, absolute topic paths, launch param-override bug, ...). **Full pitfall list and tuning notes:** [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md).
- Tuning ported from Foxy real-robot yaml: `point_filter_num=1`, `filter_size_*=0.10`, `extrinsic_est_en=true`, `pcd_save_en=false`. Effective rate ~8 Hz on Orin (was 10 Hz with default 0.50 voxel — extra CPU buys ICP correspondence density for sparse outdoor scenes).

## Active state (2026-05-10) — CFPA2 policy + Go2W wheel-skid fix

- **CFPA2 stable-challenger goal override** ([commit 0505ff0](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py)).
  `_apply_goal_policy` previously held a goal as long as the robot was making progress, never re-evaluating utility under the current map → robot kept committing to the older frontier even when a much better one appeared mid-flight. New override (gated by 3-tick streak + 2s lock-age + 1.20× score-improvement vs re-evaluated `utility(last)`) lets a freshly-seen high-IG frontier preempt without re-introducing zigzag from cluster centroid jitter. Bypasses `goal_lock_sec` since the streak gate already provides anti-thrash. Set `cfpa2_challenger_streak_required=0` to disable.

- **CFPA2 multiplicative overlap penalty for joint allocator** (same commit).
  Old `joint = a + b - λ·overlap` with λ=1.0 deducted ≤ 1.0 from IG-dominated sums in the hundreds-thousands → both robots routinely chose the same frontier region. New `joint = (a + b) × (1 − λ·overlap)` makes λ a "max % deduction when fully overlapping" (default 0.5 = up to 50% off). Scale-invariant w.r.t. IG-box / `cfpa2_w_ig` changes. Also tightened `cfpa2_sigma_overlap_m: 0 → 4` (was 2×sensor_range fallback ≈ 7m, too gradual). Live test: same-room (500+500, 2m apart) joint=520 vs different-rooms (400+350, 12m apart) joint=735 → splits decisively.

- **Go2W wheel-skid bug — router subscribed to wrong joint_states topic** ([commit 5d0e01b](src/go2w/go2w_control/scripts/go2w_hybrid_cmd_router.py#L81)).
  In legged mode the router is supposed to mirror actual wheel ω back as setpoint so kv=5 actuator brake torque stays at 0 (true freewheel). Live `ros2 topic echo` showed it publishing `[0,0,0,0]` indefinitely → `kv·(0 − ω_actual)` ≈ 13 N·m brake force per wheel under CHAMP gait → wheels skidded. Two root causes:
  1. Router default `wheel_state_topic = /mujoco_sim/joint_states` (absolute) had publisher_count=0 in single-robot sim (controller_manager runs in `/robot/`). Subscribed to dead topic → `_latest_wheel_vels` stayed `[0,0,0,0]`. Default changed to relative `joint_states` (works for single); mixed/dual launches override back to `/mujoco_sim/joint_states` explicitly.
  2. `nav_test_mujoco_fastlio.launch.py` was spawning a duplicate `go2w_hybrid_cmd_router` (the included `single_go2w_mujoco_cfpa2.launch.py` already starts one). Both shared name+ns → CHAMP saw 2× messages. Removed.

  Bonus dual-Go2W fix: per-namespace `wheel_joint_names` override (b_*-prefixed for robot_b) so robot_b's router doesn't read robot_a's wheel ω from the shared `/mujoco_sim/joint_states`.

## Active state (2026-05-05) — SE2-only direction + observability + lidar_range

- **SE2 holonomic is now the canonical and only navigation profile we tune for.**
  After 2026-05-02 validated `se2_holonomic` (SmacPlannerLattice + diff primitives + no-strafe MPPI) as the operator-preferred behavior, all subsequent tuning effort (frontier reachability, costmap inflation, MPPI critics, exploration metrics, stop conditions) targets THIS profile and no other. The legacy `holonomic_profile=off` (diff-drive Reeds-Shepp) and `omni_2d` (SmacPlanner2D + MPPI Omni) remain in [`real_single.launch.py`](src/go2w/go2w_real_bringup/launch/real_single.launch.py) and [`real_autonomy.sh`](scripts/real/real_autonomy.sh) as **escape hatches only** — not maintained for daily nav and not recipients of new tuning. New operators / sessions should default to SE2.

- **Streamlined entry: [`scripts/real/real_autonomy_se2.sh`](scripts/real/real_autonomy_se2.sh).**
  Thin wrapper over `real_autonomy.sh` that bakes in `holonomic_profile=se2_holonomic` and forwards a curated subset of flags (`robot, slam, oa, execute, lidar_range, manual, record`). Daily ops should call this; profile-selection complexity stays out of operator memory. The full surface (off / omni_2d / etc.) remains accessible via `real_autonomy.sh` for ad-hoc comparison runs.

- **`lidar_range` launch param (default 8.0 m).**
  Threaded through `real_autonomy.sh → real_single.launch.py → real_bringup_core.launch.py`; controls octomap_server raytrace + `pointcloud_to_laserscan` `range_max` simultaneously. Lower (e.g. 4.0 m) for cluttered indoor scenes where far-range obstacles introduce noise; raise for open spaces. Independent of Fast-LIO's `det_range: 100.0` (SLAM odometry quality, not perception).

- **Exploration observability + explicit stop condition shipped.**
  [`exploration_metrics_logger.py`](src/go2w/go2w_observability/scripts/exploration_metrics_logger.py) is now the central event aggregator. Adds:
  1. **Structured event log** to stdout + `$ROS_LOG_SESSION_DIR/exploration_events_*.log` — one line per significant event with absolute timestamp + Δ from previous event. Sources: `/<ns>/exploration_status`, `/<ns>/goal_pose`, `/<ns>/behavior_tree_log` (plan timing via BT-internal timestamps, dedupes recovery-branch `ComputePathToPose` duplicates), `/<ns>/recovery_event`, `/cmd_vel`.
  2. **30 s rolling summary** — `plan_ms[p50/p95/max]`, `plan_ok=N/M (P%)`, `goals[done/abort_p/abort_c]`, `ttm_avg`, `coverage Δ%`, `status`.
  3. **Stop trigger** — publishes latched `/<ns>/exploration_complete` (with reason) + cancels `navigate_to_pose` action + zero-cmd_vel pulse, when either:
     - `consec_no_reachable_threshold=3` ticks of CFPA2's `no_reachable` status, or
     - coverage Δ < `coverage_stagnant_threshold_pct=0.5%` over `coverage_stagnant_window_sec=30s`.

  CFPA2 ([`cfpa2_single_robot_node.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py)) gained a `_status_pub` on `/<ns>/exploration_status` (states: `searching | executing | no_reachable | no_frontiers | paused`) and a subscriber to `/<ns>/exploration_complete` that latches a `_paused` flag. `verbose_logs: false` (default) suppresses the per-tick `_log_no_goal_debug` / `_maybe_log_summary` spam — only state-change INFO lines remain. [`stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) publishes `/<ns>/recovery_event` (`stuck_detected | backup_started | backup_done | backup_aborted | backup_unavailable`) so recovery activity threads through the same event stream.

- **CFPA2 ↔ Nav2 threshold alignment fix.**
  Discovered a 0.40–0.60 m dead band where Nav2's `xy_goal_tolerance: 0.40` declared SUCCEEDED but CFPA2's `switch_min_dist: 0.60` still considered the goal "in flight" — same goal got republished, bridge filtered it as unchanged, robot sat idle. Fix: [`cfpa2_single_robot.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml) `switch_min_dist: 0.60 → 0.45` and added `reached_blacklist_dist: 0.45` so CFPA2's notion of "reached" sits above Nav2's. Long-term fix (action-status listener for ground-truth SUCCEEDED/ABORTED) is documented but not yet implemented.

- **CFPA2 reachability uses an inflation-blind BFS** ([`cfpa2_coordinator_node.py:1208`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#L1208)).
  `_distance_transform` walks 4-connected through cells where `_is_free()` is True (i.e. `0 ≤ v < occ_thresh` AND `v != -1`). It does NOT account for inflation, footprint, or kinematics — so it can mark a frontier reachable that Nav2 then cannot path through. For now this is documented as a known gap; the planned fix is to feed CFPA2 from `/{ns}/global_costmap/costmap` (inflated) instead of `/{ns}/map` (raw octomap projection).

## Active state (2026-05-02) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-05-02--go2w-real-nav2-profile-split--no-crab-se2-tuning).
- Headline: 3-profile runtime matrix (`off` / `omni_2d` / `se2_holonomic`) for Go2W real Nav2; finding that `SmacPlanner2D` is XY-only (one angle bin in Node2D); final SE2 tuning forces no-strafe execution (`MPPI motion_model=DiffDrive`, lattice diff primitives, forward+yaw bias). Sim parity added via [`nav2_se2_holonomic_overlay_sim.yaml`](src/go2w/go2w_config/config/nav/nav2_se2_holonomic_overlay_sim.yaml). 2026-05-05 superseded this with "SE2-only" — the older `off` / `omni_2d` profiles are escape hatches now.

## Active state (2026-04-30) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson).
- Headline: onboard SLAM split succeeded (Jetson Foxy + laptop Humble via cross-host DDS); the 13 GB-per-node memory blow-up was resolved with `ulimit -v`; compatibility notes documented for future Humble migration.
- Canonical runbook remains [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson).

## Active state (2026-04-29) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-29--dual-robot-nav2-mppi-migration).
- Headline: dual-robot stack migrated to Nav2 MPPI/SmacHybrid; fixed wheel brake bug in vendored `mujoco_ros2_control`; added `stuck_watchdog`; unified sim/real TF+odom path through `fast_lio_tf_adapter`.
- Deep narrative and rationale remain in [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md).

## Active state (2026-04-26) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack).
- Headline: A* retired for dual-robot runs in favor of FAR + safety stack (`pivot-lock`, `path_safety_filter`, `cmd_vel_safety_shield`), with documented deadlock tradeoffs and TF remap pitfalls.
- Consolidated context remains in [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) and [docs/claude/debug_notes.md](docs/claude/debug_notes.md).

## Skill-API detail docs

| Topic | Doc |
|---|---|
| Nav stack benchmarking (Phase 5), config A, iteration logs | [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) |
| Fast-LIO2 / Cartographer A/B, LiDAR options, demo scenes | [docs/claude/slam_and_scenes.md](docs/claude/slam_and_scenes.md) |
| Cross-cutting debugging gotchas (QoS, zombies, MuJoCo quirks) | [docs/claude/debug_notes.md](docs/claude/debug_notes.md) |
| **Gazebo vs MuJoCo — why stack works in Gazebo, MuJoCo matches real life** | [docs/claude/sim_comparison.md](docs/claude/sim_comparison.md) |
| **Real Go2W / Go2 — connect modes, SLAM A/B, Mid-360 calib, 10-layer bug chain** | [docs/claude/real_robot.md](docs/claude/real_robot.md) |
| **Go2 (non-W) integration — Menagerie MJCF, CHAMP shipped, real CMU TARE → localPlanner (FAR bypassed), RL scaffold in place** | [docs/claude/go2_integration.md](docs/claude/go2_integration.md) |
| **Dual-robot FAR safety stack (2026-04-26): pivot-lock + path_safety_filter + cmd_vel_safety_shield. Why A\* was retired for dual.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack) |
| **Nav2 MPPI migration journey (2026-04-29): A\* → FAR → MPPI, brake bug, freewheel, stuck_watchdog, TF/SLAM chain. Two real-robot blockers.** | [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md) |
| **Onboard SLAM split (2026-04-30): Fast-LIO + Livox onto Go2 Jetson Foxy. 5 source patches, the 13-GB-per-node bloat, the `ulimit -v` fix.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson) + [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson) |
| **Noetic FAST-LIO2 onboard (2026-05-11): ROS 1 native build of HKU FAST-LIO + Livox driver on Jetson for gbplanner3. 11 patches, conda-poison guard, X11-free Open3D streaming.** | [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md) |
| **Point-LIO + gbplanner3 onboard (2026-05-13): switched SLAM from FAST-LIO2 → Point-LIO (iVox vs ikd-tree fixes 36% rate decay), full gbplanner3 stack built on Jetson, 8 build pitfalls (src/src/, COLCON_IGNORE, gflags ExternalProject, BT XML path depth, v2→v3 config schema, ...).** | [docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md) |
| **nvblox 3D frontier exploration on demo_ramp (2026-05-13): 8 bug fixes wiring nvblox_frontend mapper → Nav2 StaticLayer → CFPA2 3D IG. Includes ground-clutter filter, ramp-vs-stairs discretization fix, RewrittenYaml topic-rewrite gotcha. Open analysis: current "3D" only changes IG, not frontier search; proposes voxel→ground projection.** | [docs/claude/nvblox_3d_frontier.md](docs/claude/nvblox_3d_frontier.md) |
| **GBPlanner3 / OmniPlanner sim-side integration (2026-05-13): ros1_bridge gotchas (latched /tf_static drop), voxblox + elevation_mapping config alignment with NTNU UAS UGV ref, host-pipe spam filter, the unsolved voxblox-doesn't-populate wall, sim-only vs. Jetson-portable artifact split.** | [docs/claude/gbplanner3_integration.md](docs/claude/gbplanner3_integration.md) |
| **3D frontier exploration unstuck (2026-05-14): 6 compounding bugs in `nav_test_3d_explore.sh`. The load-bearing one: mean-of-frontier-voxels centroid for a SHELL-shaped frontier lands at the ROBOT, so the goal = current pose → no motion ever. Plus octomap-style projection (drop fan-fill, query nvblox 3D state direct), world-fixed 40m trav_grid with persistent cls, z-band UNK filter, geometric blind disk, ClusterTracker non-actionable, MuJoCo lidar sim density.** | [docs/claude/3d_frontier_debugging.md](docs/claude/3d_frontier_debugging.md) |
| **ETH trav pipeline (2026-05-14): elevation_mapping_cupy + grid_map_filters + OccupancyGrid adapter replacing the broken 6-step 2D projection. 7-phase rewrite plan, 4 YAML/param gotchas discovered during integration, smoke-tested at 12 layers + 9 Hz.** | [docs/claude/plans/2026-05-14-trav-grid-rewrite.md](docs/claude/plans/2026-05-14-trav-grid-rewrite.md) + [docs/claude/eth_elevation_mapping_design.md](docs/claude/eth_elevation_mapping_design.md) |
| **Open problem (2026-05-15): Go2W tips at ramp foot / platform cliff. All static-cost mitigations (trav thresholds, height-cost layer, inflation halo, ramp_safe envelope) applied; tipping is dynamic-stability failure unobservable from 2.5D. 5 candidate next steps listed.** | [docs/claude/ramp_tipover_open_problem.md](docs/claude/ramp_tipover_open_problem.md) |
| **Jetson Orin Nano HIL bag-replay (2026-05-18 late night): full autonomy stack stress-tested on weaker bench board (6×A78AE @ 1.5 GHz, 8 GB) as proxy for real Go2's Orin NX 16GB. fast_lio sustains 10 Hz / RSS 192 MB / CPU 9-12% on 170s real-walk bag, RTF=1.07 (real-time confirmed). Point-LIO ROS 2 port abandoned: SIGSEGV race at IMU Init 100% (~50% rate) + SLAM divergence on Mid-360 walking data. 8 fixes shipped: DDS isolation via ROS_DOMAIN_ID=42, livox_ros_driver2_msgs stub package, bag-play workspace-source preflight, `--clock`+`use_sim_time=true`, multi-pass kill, Point-LIO `/aft_mapped_to_init` remap, staggered TimerActions + respawn, rclpy `warn_throttle` migration. Real bottlenecks identified: **CFPA2 Python tick p95 1.4s** + **grid_map_to_occupancy 0.59 Hz** — single-core Python on ARM. Refuted hypotheses: concurrent record overhead (38 MB/s disk write didn't dent fast_lio), fast_lio degradation (no ikd-tree growth observed in 170s with `pcd_save_en=false`). Likely culprit for onboard 5.7 Hz baseline: ros1_bridge serialization overhead.** | This file's "Active state (2026-05-18 late night)" entry |
| **CFPA2 pure C++ port + hexagonal isolation (2026-05-19): 1,400 LOC algorithm body now ROS-independent through `core::IClock` / `ILogger` / `IGoalPublisher` / `IVisualizer` interfaces + POD `core::Grid` / `OdomXY`; ROS-2-specific code confined to `ros2/*.hpp` adapters. Jetson tick p95 1376 ms (Python) → 1.1 ms (C++) = ~1250× speedup. Two executables `cfpa2_{coordinator,single_robot}_node_cpp` install side-by-side with Python entry points; flip via launch arg `cfpa2_executable_suffix:=_cpp`. Noetic port reduced from "rewrite 1,400 LOC" to "swap one adapter directory + edit ctor" (~5 h estimated).** | [docs/claude/noetic_port_checklist.md](docs/claude/noetic_port_checklist.md) + this file's "Active state (2026-05-19)" entry |
| **Native ROS 2 CUDA-MPPI plugin (2026-05-19 night): `nav2_mppi_controller_cuda_plugin::CudaMPPIController` — `nav2_core::Controller` subclass injecting the 11-kernel GPU backend into Nav2 Humble's `controller_server` directly. Same `.cu` sources as Noetic port via `#ifdef NAV_ALGO_MPPI_CUDA_USE_NAV2`. Key bugs: pluginlib separator must be `::` not `/`; `nav2_package()` breaks nvcc; `CUDAToolkit_INCLUDE_DIRS` needed for host `.cpp`. Verified live: 8 critics + CUDA backend ENABLED on configure. `nav_test_cuda_mppi.sh` is the entry point.** | This file's "Active state (2026-05-19 night)" entry |
| **Nav2 SmacLattice + MPPIController ROS 1 Noetic port + full GPU MPPI pipeline (2026-05-19 evening): 13K LOC algorithm body byte-identical to Nav2 Humble upstream (only include-path rewrites + `compat.hpp` injection); 380-line shim bridges rclcpp logging, msg type aliases, costmap_2d namespace, FootprintCollisionChecker re-impl, LifecycleNode → ros::NodeHandle stub. 5 catkin packages: `nav_algo_{core, smac_ros1, mppi_ros1, bringup, mppi_cuda}`. End-to-end sim test in `ros:noetic-ros-core` Docker: `cmd_vel = (0.30, 0.0)` at vx_max towards goal. **All 11 MPPI kernels** (integrate + 8 critics incl. ObstaclesCritic footprint Bresenham + cost-shape + softmax + weighted-avg) validated against CPU references — `max \|Δ\|` across all 11 within 1e-4, several at fp32 ε ~1e-7 or bit-exact 0. Total MPPI hot loop CPU 12 ms → GPU 0.25 ms = ~50× on RTX 4050 (Orin NX projected ~80×). CudaBackend wired into Optimizer via `ICudaBackend` injection (nav_algo_core remains CUDA-free at build level); 200+ live `Optimizer::optimize()` dispatches verified via probe instrumentation. v1 limitations documented: host-side critic gates skipped, critic weights hardcoded to yaml — both follow-ups planned. Companion to CFPA2 C++ port: full autonomy stack now Noetic-deployable.** | This file's "Active state (2026-05-19 evening)" entry |
| **Go2W real Nav2 profile split (2026-05-02): `off` / `omni_2d` / `se2_holonomic`, SmacPlanner2D XY-only finding, no-crab final SE2 tuning.** | This file's "Active state (2026-05-02)" entry |

## Scripts layout

```
scripts/
├── launch/    user-invoked entry points (nav_test_*, vlm_demo)
├── bench/     multi-trial PASS-criterion runners + session_reporter
├── runtime/   ROS 2 nodes started by launch files (policy, checkers, supervisors)
├── debug/     observe-a-running-sim tools (far_monitor, vlm_debug_web, …)
├── ops/       one-shot ops & dev utilities (reset_vgraph, test_lidar, sync_to_main)
├── real/      real-robot only (unchanged grouping)
└── common_logging.sh
```

## Quick launch

```bash
# Single-robot nav with CUDA-accelerated MPPI (CudaMPPIController, use_cuda:true default)
./scripts/launch/nav_test_cuda_mppi.sh                                # Go2W (has_wheels=true)
./scripts/launch/nav_test_cuda_mppi.sh has_wheels:=false              # Go2 walking
./scripts/launch/nav_test_cuda_mppi.sh gui:=false rviz:=false         # headless
./scripts/launch/nav_test_cuda_mppi.sh nav_costmap_mode:=3d           # ETH trav grid

# Single-robot Go2W nav smoke test (Nav2 MPPI + SE2 by default, same as above since yaml default is CUDA)
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=true rviz:=true
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=false           # headless

# Nav benchmark (FAR baseline) — 5-trial / 10-trial
./scripts/bench/benchmark_far_nav.sh
NUM_TRIALS=10 DURATION_SEC=120 OUT_DIR=/tmp/cfgA_10 ./scripts/bench/benchmark_far_nav.sh
./scripts/bench/benchmark_fastlio.sh                                  # Fast-LIO + MID-360

# Demo2 / LRC maze
./scripts/launch/nav_test_demo2.sh gui:=false
./scripts/launch/nav_test_lrc_maze.sh

# VLM exploration demo (Phase 1) — uses nav2_mppi
./scripts/launch/vlm_demo_mujoco.sh

# Heterogeneous dual (Go2W + Go2 + demo3_mixed + CFPA2 coord, nav2_mppi)
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far  # both FAR

# Go2 (no-wheel) sim — CHAMP locomotion, demo1 12×8 m / demo3 24×16 m
./scripts/launch/nav_test_go2.sh gui:=true rviz:=true                 # walk + FAR smoke
./scripts/launch/nav_test_go2_demo3.sh gui:=true rviz:=true           # larger scene
./scripts/bench/benchmark_go2.sh                                      # 5-trial PASS check
./scripts/bench/benchmark_go2_demo3.sh                                # same on demo3
./scripts/launch/nav_test_go2_tare_real.sh gui:=true rviz:=true       # CMU TARE → localPlanner
./scripts/bench/benchmark_go2_tare.sh                                 # 10-trial TARE benchmark
./scripts/launch/nav_test_go2.sh rl_policy:=true                      # RL (experimental, see go2_integration.md)

# Real robot (Go2W) — RECOMMENDED entry as of 2026-05-05:
#   SE2 holonomic baked in, oa=false (sport API direct), curated flag surface.
./scripts/real/real_autonomy_se2.sh                                   # default Go2W SE2
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360                # Mid-360 + Fast-LIO
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360 lidar_range=4.0
./scripts/real/real_autonomy_se2.sh stop                              # kill everything

# Legacy multi-profile real launcher (escape hatches for ad-hoc comparison):
./scripts/real/real_autonomy.sh                                       # Cartographer + L1 default
./scripts/real/real_autonomy.sh slam=fastlio_mid360
./scripts/real/real_autonomy.sh oa=false holonomic_profile=off        # diff-drive Reeds-Shepp
./scripts/real/real_autonomy.sh oa=false holonomic_profile=omni_2d    # SmacPlanner2D + MPPI Omni
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far
./scripts/real/real_autonomy_go2.sh                                   # Go2 (no wheel), nav2_mppi
./scripts/real/real_autonomy_go2.sh slam=fastlio_mid360 nav=far
# Real CMU TARE → localPlanner direct (FAR unwired, watchdog armed).
# oa=false is REQUIRED — default (oa=true) routes Move to /api/obstacles_avoid/request
# which needs manual mode pre-arm; oa=false sends to /api/sport/request (api_id=1008).
./scripts/real/real_autonomy.sh robot=go2 slam=fastlio_mid360 nav=tare_real oa=false
./scripts/real/real_autonomy.sh stop                                  # kill everything real-robot

# Flip CFPA2 Python entry point → pure-C++ binary on any existing launch
./scripts/launch/nav_test_3d_explore.sh cfpa2_executable_suffix:=_cpp
./scripts/launch/nav_test_slam_ops2_v4_go2.sh cfpa2_executable_suffix:=_cpp
ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py cfpa2_executable_suffix:=_cpp

# Build + smoke-test the Nav2 ROS 1 port (SmacLattice + MPPI) in Noetic Docker.
# Image nav_algo:build_env_cuda is cached locally (3 GB, CUDA 12.6 + xtensor +
# nav_core + costmap_2d). Builds all 5 packages, runs the move_base integration
# test on a synthetic OccupancyGrid + reports cmd_vel.
docker run --rm --gpus all \
  -v $PWD/src/vendor/nav_algo_ros1:/ws/src/nav_algo_ros1:ro \
  nav_algo:build_env_cuda \
  bash -c 'cp -r /ws/src/nav_algo_ros1 /ws_rw/src/ && cd /ws_rw && \
           source /opt/ros/noetic/setup.bash && \
           catkin_make_isolated -DCMAKE_BUILD_TYPE=Release && \
           bash /ws_rw/src/nav_algo_ros1/nav_algo_bringup/test/integration_test.sh'

# CUDA per-kernel CPU-vs-GPU benchmarks (RTX 4050 / Orin sm_87 compatible).
# integrate_bench    — integrateStateVelocities (139× on RTX 4050)
# critics_bench      — all 8 critics + 9th case for ObstaclesCritic footprint mode
# control_update_bench — cost-shape + softmax + weighted-avg
docker run --rm --gpus all -v $PWD/src/vendor/nav_algo_ros1:/ws/src/nav_algo_ros1:ro \
  nav_algo:build_env_cuda \
  /ws_rw/devel_isolated/nav_algo_mppi_cuda/lib/nav_algo_mppi_cuda/integrate_bench
docker run --rm --gpus all -v $PWD/src/vendor/nav_algo_ros1:/ws/src/nav_algo_ros1:ro \
  nav_algo:build_env_cuda \
  /ws_rw/devel_isolated/nav_algo_mppi_cuda/lib/nav_algo_mppi_cuda/critics_bench
docker run --rm --gpus all -v $PWD/src/vendor/nav_algo_ros1:/ws/src/nav_algo_ros1:ro \
  nav_algo:build_env_cuda \
  /ws_rw/devel_isolated/nav_algo_mppi_cuda/lib/nav_algo_mppi_cuda/control_update_bench

# Toggle GPU MPPI on/off in the integration test (yaml param):
#   use_cuda: true  → CudaBackend takes over Optimizer::optimize (the default)
#   use_cuda: false → xtensor CPU path (silent fallback when CUDA not built)
# Either flips between full GPU pipeline and the original Humble-port CPU path
# without changing any other yaml, controller frequency, or plugin loading.
sed -i 's/use_cuda:.*$/use_cuda: false/' \
  src/vendor/nav_algo_ros1/nav_algo_bringup/config/mppi_controller_params.yaml  # CPU
sed -i 's/use_cuda:.*$/use_cuda: true/'  \
  src/vendor/nav_algo_ros1/nav_algo_bringup/config/mppi_controller_params.yaml  # GPU
```

Debug dashboard:
- VLM exploration: <http://localhost:8501> (auto-starts)

## Build

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Full build
touch src/mtare_ros1_ws/COLCON_IGNORE
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
```

YAML + Python changes are instant via symlink-install; C++ requires rebuild.

## Repo layout

```
src/
  go2w/                             Go2W platform packages
    go2_gazebo_sim/                   MJCF/world + launch files
    mujoco_sensor_bridge/             MuJoCo sensor nodes
    go2w_control/                     Hybrid cmd_vel router (legged + wheel mux)
    go2w_nav/                         Safety utilities (collision_monitor, cmd_vel_safety_shield, …)
    go2w_perception/ go2w_config/     QoS bridge, robot_self_filter, configs, sub-launches
    unitree_go2w_ros2/                Unitree ROS 2 integration
  collaborative_exploration/
    cfpa2_collaborative_autonomy/     CFPA2 frontier allocator (single + dual joint allocator)
    dynamic_scene_filter/             Dynamic obstacle filter (peer body + temporal voxel)
    slam_backend_adapters/            Fast-LIO ↔ Nav2 adapters
  vendor/
    fast_lio/                         Fast-LIO2 SLAM
    autonomy_stack_go2/               CMU stack (FAR, terrain analysis)
    mujoco_ros2_control/              DFKI MuJoCo HW interface
    far_planner/                      FAR global planner
  vlm_explorer/                     VLM-in-the-loop exploration (Phase 1)
scripts/                            Launch scripts, benchmark runners, utilities
config/                             DDS config (fastdds_no_shm.xml)
docs/claude/                        Detailed skill-API docs (this index)
```

## Nav backends

Switchable via `nav_backend:=` at launch time. Two production backends now (legacy `astar` / `default` / `reactive` / `mppi_nav_node` were retired in commit 5c46a51).

| Backend | Planner | Note |
|---|---|---|
| `nav2_mppi` | `nav2_planner` (SmacPlannerHybrid REEDS_SHEPP / SmacPlannerLattice for SE2) + `nav2_controller` (MPPIController) + `nav2_behaviors` + `nav2_bt_navigator` + `nav2_lifecycle_manager` | **Default for sim and real.** Per-platform yaml: [`nav2_go2w_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml) for Go2W, [`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml) for Go2. Real Go2W supports overlay profile selection via `holonomic_profile`: `off` (diff-drive Reeds-Shepp, escape hatch), `omni_2d` (`SmacPlanner2D` + MPPI Omni, escape hatch), `se2_holonomic` (`SmacPlannerLattice` + forward/pivot MPPI, no strafe — **canonical**). Outer-loop `stuck_watchdog` per robot. CFPA2 `way_point` is bridged to `goal_pose` via `cfpa2_to_nav2_bridge`. |
| `far` | CMU autonomy stack | Terrain analysis + FAR V-graph + path follower (see nav_benchmarks.md). Used for benchmarking comparison and Go2 TARE exploration. |

## Golden rules (must-follow across all work)

1. **Always `use_sim_time: true`** for all nodes in MuJoCo or Gazebo. Mixed time domains corrupt maps.
2. **Never use stale TF fallback.** Drop the scan on TF failure; don't use `tf2::TimePointZero`.
3. **Each scan painted exactly once.** Clear `last_scan_` after processing in mappers.
4. **Dual-robot TF must be namespaced.** Remap `/tf` → `/{ns}/tf` for all nodes.
5. **DDS config matters.** `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS.
6. **Verify with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.
7. **Kill zombie MuJoCo before re-launch** (see [debug_notes.md](docs/claude/debug_notes.md)).
8. **Benchmark PASS criterion** = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. 5 trials is too small for reliability claims; use ≥10.
9. **Supervisor panic = any-button override (real robot).** Any press on the Unitree BT pad latches a 5 s window: auto `cmd_vel` blocked, FAR disarmed, sticks drive directly. See [docs/claude/real_robot.md](docs/claude/real_robot.md#supervisor-panic-override-any-button-emergency).
10. **Any node doing TF lookup in dual-robot setup MUST have `tf_remaps`.** Without `("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")`, the node's TF buffer subscribes the global `/tf` (empty in our namespaced setup) and every lookup silently fails — no error log, just stale data or `passthrough_no_tf` status. Discovered the hard way 2026-04-26: path_safety_filter + cmd_vel_safety_shield were inert for hours because of this. **Verify lookups work via `ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/{ns}/tf -r /tf_static:=/{ns}/tf_static`**.
11. **CMU's `vehicle` frame is NOT in our SLAM tree by default.** CMU only publishes `sensor → vehicle` (static); `sensor` itself isn't connected to `map` (we have `map → sensor_at_scan`, not `sensor`). So `lookup_transform(map, vehicle)` fails. The mixed launch bridges this with a `base_link → vehicle` static publisher per namespace; nodes reading paths in vehicle frame should also accept a `base_frame_fallback` parameter.
12. **Multi-layer safety stacks deadlock easily.** CFPA2 pivot-lock (refuses goal change) + cmd_vel shield (kills ω) + path_safety_filter (rejects path) can all latch a robot in place if held goal demands rotation it can't execute. Always provide a max-hold/timeout escape valve (e.g. `pivot_lock_max_hold_sec`) on each stateful safety layer. Verify each layer's status topic, NOT just the absence of motion — a robot frozen by 3 layers stacked looks identical to "stuck planner" in /nav_status alone.
13. **`peer_pose_stale_sec` must be generous in sim.** sim_time and wall-clock-stamped messages can drift several seconds during startup; a strict 0.3s threshold rejects fresh peer poses → self_filter publishes scans untouched → peer body becomes a permanent imprint in /map. Default 5.0s in dual-robot launches; tighten on real robot only after verifying timestamp alignment.
14. **MPPI's effective rejection radius = `robot_radius` + `collision_margin_distance`.** With `consider_footprint: false` (the default-because-old-comment-said-it-throws), MPPI treats the robot as a circle and adds margin on top. demo3_mixed has 0.425 m corridors; with `robot_radius=0.40 + margin=0.20 = 0.60 m` MPPI permanently rejected forward motion. **Always set `consider_footprint: true` + a polygon `footprint:` on both costmaps**, then drop `collision_margin_distance` to 0.03 m. The "throws at configure" comment was a yaml-parse bug, not a humble-1.1.20 platform bug.
15. **Footprint changes ripple through CFPA2 frontier filters** ([cfpa2_coordinator.yaml](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_coordinator.yaml)). When you tighten footprint, you typically need to *loosen* `cfpa2_frontier_obstacle_clearance_m`, `cfpa2_frontier_unknown_check_radius_m`, and `cfpa2_frontier_min_cluster_area_m2` proportionally, otherwise late-stage exploration falsely declares "complete" with significant unknown remaining. CFPA2 has **no parameter callback** (`add_on_set_parameters_callback` is not registered), so `ros2 param set` only updates the param store; cached `self.cfpa2_*` values stay frozen. Edit yaml + restart the node.
16. **Sim/real share the same TF + odom data path via `fast_lio_tf_adapter`.** This was *not* the case before 2026-04-29: sim relied on `mujoco_odom_bridge` writing GT directly to TF (bypassing the EKF chain), and real had nothing to take over because CHAMP's `state_estimation_node` outputs `7.8e+34` NaN that starves the EKF. Now [`scripts/runtime/fast_lio_tf_adapter.py`](scripts/runtime/fast_lio_tf_adapter.py) is the single owner of `odom → base_link` TF and `/<ns>/odom/nav` topic, sourced from Fast-LIO's `/<ns>/Odometry`. **`mujoco_odom_bridge.publish_tf` must stay `false`** (or it competes with the adapter on the same TF link). Real-bot drops in unchanged; only `bootstrap_from_gt:=false` differs (no GT to align to).
17. **Don't blame the architecture before grepping the upstream.** Half a day was lost mis-locating the wheel brake bug at the router / MPPI / footprint level when the root cause was a 2-line bug in vendored `mujoco_ros2_control` (the VELOCITY actuator branch never updated `last_command`, so once the controller commanded 0 once, ctrl was never refreshed and stayed at the previous setpoint). When data clearly shows a chain breaking and "everyone says they're publishing the right thing", stop tuning higher layers and read the lowest layer's source.
18. **Outer-loop stuck recovery is needed because MPPI/DWB/FAR rarely *report* failure.** They emit (v ≈ 0, ω ≈ 0) and self-report happy. Nav2's BT recovery only fires on a controller-reported failure → never triggers in this scenario. [`scripts/runtime/stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) is the watchdog: 10 s no-motion + active goal → Nav2 BackUp action → republish goal. Per-namespace, real-robot-compatible. Caveat: BackUp itself collision-checks `simulate_ahead_time × backup_speed` of clearance (≈ 0.20 m), so a robot wedged between two walls still fails recovery — last-resort raw-cmd_vel pulse not yet wired.
19. **`SmacPlanner2D` is XY-only; do not expect it to solve heading-dependent footprint-fit maneuvers.** If the requirement is "choose body orientation to pass anisotropic narrow geometry", use an SE2 planner (`SmacPlannerHybrid` / `SmacPlannerLattice`) and tune controller execution policy separately.
20. **Missing file errors under `install/.../share/<pkg>/...` usually mean stale install artifacts, not bad launch paths.** New YAML under `src/` is invisible to `get_package_share_directory()` until that package is rebuilt (`colcon build --symlink-install --packages-select <pkg>`). This exact failure happened with `nav2_go2w_real_omni_overlay.yaml` on 2026-05-01.
21. **Don't hardcode absolute topic paths to controller_manager-namespaced state sources.** JointStateBroadcaster (and most `controller_manager`-loaded broadcasters) publish to `<cm_ns>/joint_states`, where `cm_ns` varies per launch: single-robot sim runs `cm_ns=/robot/`, mixed/dual sims run `cm_ns=/mujoco_sim/`, real robots run yet another. A node hardcoding `wheel_state_topic=/mujoco_sim/joint_states` worked silently in mixed/dual but had `publisher_count=0` in single → router published `[0,0,0,0]` wheel commands → kv=5 actuator brake-locked the wheels under CHAMP gait → wheel-skid bug, hidden for months (2026-05-10 fix). Use a relative default that picks up the per-namespace topic, and override in launches whose `cm_ns` is elsewhere. **Verify with `ros2 topic info <topic> -v` that publisher_count > 0 BEFORE assuming the node is wired.**
22. **CFPA2 algorithm body is ROS-independent (Phase D/E hexagonal isolation, 2026-05-19).** New algorithm code should go through `cfpa2::core::IClock` / `ILogger` / `IGoalPublisher` / `IVisualizer` and the `CFPA2_LOG_INFO/WARN/ERROR` macros — NEVER call `get_clock()->now()`, `RCLCPP_*`, or touch `nav_msgs::msg::*` directly from the algorithm body. The ROS-specific implementations live in `include/cfpa2_collaborative_autonomy/ros2/` and a future `ros1/` mirror; algorithm code should compile cleanly when `ros2/` is removed and only `core/` is in scope. The Noetic port (Orin NX 16 GB target on real Go2) depends on this discipline.
23. **ROS 2 algorithm code can be ported byte-for-byte to ROS 1 via a thin compat shim, NOT a rewrite (2026-05-19 evening).** Nav2 Humble's SmacPlanner + MPPI (~13K LOC algorithm body) compiles cleanly on Noetic by including one [`compat.hpp`](src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/compat.hpp) that provides: rclcpp logging macros → ROS_* (drop logger handle arg), `geometry_msgs::msg::*` → `geometry_msgs::*` type aliases (ROS 1 & 2 msg fields are identical), `nav2_costmap_2d::*` → `costmap_2d::*` (Nav2 was forked from ROS 1 nav stack — same Costmap2D), and a few stubs (`rclcpp::Time/Duration/Clock/LifecycleNode`). The two non-trivial re-implementations (`FootprintCollisionChecker` Bresenham + `geometry_utils::{first_after_integrated_distance, min_by}`) must be traced cell-by-cell against upstream to guarantee math equivalence — empirically ~1 h per helper. The other ~99% of files diff only on `#include` path renames. Same approach works for any "compat shim is feasible" Nav2/ROS 2 port. Reference impl: [`src/vendor/nav_algo_ros1/`](src/vendor/nav_algo_ros1/). Math-equivalence audit is a precondition — see the "Equivalence audit" subsection of the 2026-05-19 evening entry for the structured pass.
24. **GPU acceleration of ported algorithms goes through an `ICudaBackend` injection point, NOT a build-time fork (2026-05-19 evening).** When CUDA-accelerating a CPU algorithm (e.g. Nav2 MPPI), the algorithm package stays CUDA-free: define a pure-virtual `ICudaBackend` interface in the algorithm header, hold an `ICudaBackend*` member in the dispatcher class (e.g. `mppi::Optimizer`), and let the dispatch hook be `if (backend_) backend_->doIt(*this); else cpu_path();`. The concrete CUDA implementation lives in a separate package that depends on the algorithm package (not the reverse). Plugin / front-end packages do `find_package(<cuda_pkg> QUIET)` and define a `HAS_CUDA` flag when present — when absent, plugin compiles + runs on CPU silently. This keeps the no-GPU dev machine happy and makes CUDA truly opt-in at runtime via a single yaml flag (`use_cuda: true`). Reference: `nav_algo_core::mppi::ICudaBackend` + `nav_algo_mppi_cuda::CudaBackend` + `MPPIControllerROS::initialize()`'s use_cuda branch. The probe-file pattern (`/tmp/cuda_backend_{ctor,optimize}` from the v1 commit) is cheap operational monitoring that confirms which path actually ran when logs are ambiguous.

## Communication style

- Fast and direct. Short questions expect immediate, precise answers.
- Show work but don't narrate it. Run commands, make changes, report what happened.
- When told "still not working", don't repeat the same fix — go deeper.
- Respect the maintainer's hypotheses. When they say "I AM certain it's due to X", treat as strong signal; validate or disprove with evidence, not conjecture.

## Environment

- Python 3.10 (micromamba `cmu_env`)
- ROS 2 Humble (`/opt/ros/humble/`)
- Build: colcon + ament_cmake / ament_python
- DDS: FastDDS (sim), CycloneDDS (real robot)
- Sim: MuJoCo 3.6.0 (pip) + DFKI mujoco_ros2_control
- VLM API: xAI Grok-4-1-fast-non-reasoning, key in `.env.xai`
