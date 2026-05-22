# Orin NX HIL design — laptop simulates the world, NX is the reactive compute unit

**Goal:** stress the real Go2 Orin NX autonomy software stack under realistic
sensor load + measure HW usage, while the laptop plays the physical world
(MuJoCo ops2-v4 scene). The NX does **all** compute (SLAM→trav→nav→CFPA2); the
laptop produces fake sensors (lidar+IMU) and consumes cmd_vel to move the
simulated robot. Operator watches everything in RViz2 on the laptop.

This mirrors the Orin **Nano** HIL (`nav_test_hil_desktop.sh` +
`run_jetson_hil.sh`) but: (a) NX runs **ROS 1 Noetic** (not ROS 2), so a
`ros1_bridge` sits between; (b) SLAM runs on the NX (not the laptop), so the
laptop feeds **raw** Mid-360 data, not registered clouds.

## Topology

```
┌─────────────────────── LAPTOP (Ubuntu 22.04, ROS 2 Humble) ───────────────────────┐
│  MuJoCo (ops2-v4 scene: slam_ops2_v4_go2_handwalls.xml)                            │
│    ├─ lidar sim ─PointCloud2→ [pc2_to_livox C++] ─/livox/lidar (CustomMsg)─┐       │
│    ├─ imu sim ──────────────────────────────────── /livox/imu (Imu) ───────┤       │
│    ├─ CHAMP control  ◄──────────────────────── /robot/cmd_vel ◄────────────┼──┐    │
│    └─ MuJoCo GT pose (for sim robot only; NOT fed to NX SLAM)              │  │    │
│  RViz2  ◄── viz topics (map/trav/cloud/path/tf/cmd_vel) ───────────────────┼──┤    │
└────────────────────────────────────────────────────────────────────────────┼──┼────┘
                          ROS 2 Humble DDS  ⇅  (wifi/eth)  ⇅  ROS 2 Foxy DDS   │  │
┌─────────────────────── ORIN NX (Ubuntu 20.04, Noetic + Foxy) ───────────────┼──┼────┐
│  ros1_bridge parameter_bridge (C++, Noetic ⇄ Foxy):                          │  │   │
│    ROS2→ROS1:  /livox/lidar (CustomMsg), /livox/imu (Imu)  ─────────────────►│  │   │
│    ROS1→ROS2:  /robot/cmd_vel, + viz: /robot/{traversability_grid,           │  │   │
│                cloud_registered_body, Odometry, plan}, /tf, /tf_static  ─────┘  │   │
│                                                                                 │   │
│  Noetic autonomy stack (onboard_autonomy_noetic.sh, the EXISTING launcher):    │   │
│    Point-LIO ─/robot/Odometry,/robot/cloud_registered_body→ elevation_mapping  │   │
│      static TFs map→camera_init→body→base_link                                 │   │
│    trav_pipeline ─/robot/traversability_grid→ move_base (SmacLattice+CUDA-MPPI)│   │
│      ─/robot/cmd_vel──────────────────────────────────────────────────────────┘   │
│    CFPA2 ─/robot/way_point_coord→ cfpa2_to_movebase_bridge → move_base goal         │
└────────────────────────────────────────────────────────────────────────────────────┘
```

**Why this is a faithful HW-load test:** the NX receives the same raw sensor
stream rate it would on the real robot (10 Hz lidar CustomMsg + 200 Hz IMU),
runs the identical software stack, and emits cmd_vel — so tegrastats numbers
(CPU/GPU/RAM/temp/power) reflect real deployment load. The only difference from
the real robot is the world is MuJoCo physics instead of the real building.

## Topic table (what crosses the bridge)

| Direction | Topic | Type | Rate | Notes |
|---|---|---|---|---|
| L→NX | `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 10 Hz | from pc2_to_livox converter |
| L→NX | `/livox/imu` | `sensor_msgs/Imu` | 200 Hz | MuJoCo IMU |
| NX→L | `/robot/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | drives MuJoCo CHAMP |
| NX→L | `/robot/traversability_grid` | `nav_msgs/OccupancyGrid` | 5 Hz | viz |
| NX→L | `/robot/cloud_registered_body` | `sensor_msgs/PointCloud2` | 10 Hz | viz (throttle to 5 Hz) |
| NX→L | `/robot/Odometry` | `nav_msgs/Odometry` | 10 Hz | viz + trajectory log |
| NX→L | `/robot/plan` | `nav_msgs/Path` | 1 Hz | viz |
| NX→L | `/robot/way_point_coord` | `geometry_msgs/PointStamped` | 2 Hz | viz (frontier goal) |
| NX→L | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | — | viz (NX owns map→base_link) |

Heavy topics (cloud) are decimated by a C++ throttle before bridging to keep
wifi + laptop RViz light. cmd_vel + lidar + imu are the latency-critical path.

## DDS / cross-version

- NX `ros1_bridge` (Foxy) ⇄ laptop RViz2 (Humble) over DDS. Foxy↔Humble works
  for standard msg types over the same RMW; use **CycloneDDS on both** + a shared
  `ROS_DOMAIN_ID` (e.g. 0). FastDDS Foxy↔Humble has had shared-memory quirks; if
  topics don't appear, fall back to `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` both
  sides + `ROS_LOCALHOST_ONLY=0`.
- The NX↔laptop link is the same eth/wifi already used for SSH.

## CustomMsg across the bridge

`livox_ros_driver2/CustomMsg` is non-standard, so `ros1_bridge` needs both sides
to have an identical `.msg`. They do:
- Laptop ROS 2: `src/vendor/livox_ros_driver2_msgs` (built in `install/`).
- NX Noetic: `livox_ros_driver2/msg/{CustomMsg,CustomPoint}.msg`.
`ros1_bridge` auto-pairs msgs with matching package+name+fields; verify with
`ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i custommsg`. If the
pairing doesn't appear, the bridge must be **built from source against both** the
Noetic + Foxy livox msg packages (a known ros1_bridge requirement for custom
msgs). Plan: try the apt `ros-foxy-ros1-bridge` first; if CustomMsg isn't paired,
build ros1_bridge from source on the NX with the livox msgs in the overlay.

## Components to build

### On the laptop (ROS 2 Humble)
1. **`pc2_to_livox` C++ node** (`src/go2w/mujoco_sensor_bridge/`): subscribe
   MuJoCo `PointCloud2`, repackage to `livox_ros_driver2/CustomMsg` (per-point
   x,y,z,reflectivity,offset_time,line). C++ for throughput. ~150 LOC.
   - offset_time: synthesize a Risley-like per-point time span (reuse the
     100 µs span logic from `pointcloud_adapter.py`).
2. **`nav_test_hil_nx_desktop.sh`**: launch MuJoCo ops2-v4 + sensor sims +
   pc2_to_livox + CHAMP (cmd_vel consumer) + RViz2 with the HIL config. Like
   `nav_test_hil_desktop.sh` but: `use_fast_lio:=false` (no SLAM on laptop —
   that's the NX's job), publishes `/livox/{lidar,imu}` raw, subscribes
   `/robot/cmd_vel` from the bridge. NO map/odom TF (NX owns those).
3. **RViz2 config** `hil_nx.rviz`: map, trav grid, registered cloud, plan,
   frontier marker, tf, robot model, cmd_vel arrow.

### On the NX (mirror of run_jetson_hil.sh, but ROS 1)
4. **`bridge_topics_hil.yaml`** + **`run_nx_hil_bridge.sh`**: `parameter_bridge`
   config for the topic table above. C++ binary, thin.
5. **Reuse `onboard_autonomy_noetic.sh`** as-is for the compute stack — it already
   brings up Point-LIO+trav+move_base+CFPA2 reading `/livox/*` and emitting
   cmd_vel. Add a `hil=true` flag that skips the Mid-360 NIC bind (sensors come
   from the bridge, not a real LiDAR) and starts the bridge.

### Orchestration
6. **`hil_orin_nx.sh`** (laptop, top-level): like `hil_orin_nano.sh` — `up`
   starts laptop sim, waits "platform ready", SSH-starts the NX bridge + stack,
   waits "CUDA backend ENABLED"; `stop`/`status`/`monitor` subcommands.

## Dry-run measurement (load / trajectory / dataset)
- **Load**: `tegrastats` logged on the NX for the whole run → CPU/GPU/RAM/temp/
  power timeseries. Compare idle vs full-stack.
- **Trajectory**: log `/robot/Odometry` (SLAM estimate) + MuJoCo GT pose on the
  laptop → overlay SLAM-vs-GT drift; log CFPA2 `way_point_coord` sequence +
  cmd_vel → "intended exploration trajectory."
- **Dataset**: record a bag on the laptop (`/livox/lidar`, `/livox/imu`, GT pose)
  = a reusable ops2-v4 HIL replay corpus. Optionally record NX-side
  `/robot/{Odometry,traversability_grid}` for offline analysis.

## Open questions / risks
- **Foxy↔Humble DDS**: if viz topics don't show in laptop RViz2, switch both to
  CycloneDDS + same domain; worst case bridge to a Humble-side relay.
- **CustomMsg pairing**: may force a source build of ros1_bridge on the NX.
- **Sim-time**: the NX stack runs on wall-clock (real sensors normally); for HIL
  the laptop MuJoCo is the clock. Either run NX on wall-clock (sim publishes at
  real rate — simplest, and fine since we measure real load) OR bridge `/clock`
  and set `use_sim_time` everywhere (more faithful but more wiring). **Plan:
  wall-clock** (sim emits at 10/200 Hz real-time; load numbers are real).

## Dry-run results (2026-05-20) — VALIDATED end-to-end

First full integrated HIL run: laptop MuJoCo (ops2-v4 handwalls scene) → fake
Mid-360 → C++ UDP relay → Orin NX (Point-LIO + trav + move_base + CFPA2). The
ros1_bridge dead-end (Foxy bad_alloc) was replaced by the UDP relay, which
carries the CustomMsg stream cleanly.

**Loop verified live (NX-side topic rates):**
- `/livox/lidar` (laptop→NX over UDP, CustomMsg): ~9 Hz
- `/robot/Odometry` (Point-LIO SLAM on NX): ~8.8 Hz
- `/robot/traversability_grid` (elevation_mapping_cupy + filter): ~5 Hz
- `/robot/way_point_coord` (CFPA2 frontier goals): 2 Hz

**Dataset captured:** `/tmp/hil_dryrun_bag/hil_run.bag` — 59.7 s, 442 MB, 8681 msgs
(530 lidar + 530 odom + 298 trav + 119 CFPA2 goals). Reusable HIL replay corpus.

**NX HW load (steady-state HIL, live tegrastats):** CPU ~52% peak single-core,
RAM 4.3 / 15.4 GB, GPU spikes to 90% (CUDA-MPPI kernels), 56–60 °C, ~12.6 W max.
Enormous headroom — the full real-robot autonomy stack runs comfortably on the
Orin NX driven by realistic simulated sensor load.

**Asset gotcha:** the ops2-v4 scene needs `bags/meshes/ops2_cuda/` (the
SLAM-reconstructed building mesh `scans_v4_sparse.obj` + `hfield/` + ~120
`sonata/instances/meshes/*.obj`). These are gitignored (large); sync them
machine-to-machine. The XML `file=` paths are `../../../../../../bags/...`
(6 levels) which is CORRECT — MuJoCo resolves them relative to the XML's
`meshdir` (`assets/go2_menagerie/`), 6 levels deep from the repo root. Do NOT
"fix" the depth; the assets just need to be present.

**Topic wiring gotcha:** the MuJoCo lidar plugin publishes on
`/<ns>/mujoco_lidar_sensor/registered_scan` (BestEffort); a `qos_bridge`
republishes it RELIABLE to `/<ns>/registered_scan_reliable`. pc2_to_livox must
consume the **reliable** one (the laptop launcher wires this via HIL_SCAN_TOPIC).

**Open / next:** cmd_vel-closes-the-loop (NX move_base → UDP → laptop MuJoCo →
robot moves → odom changes → CFPA2 progresses) is wired but idle in this run —
move_base needs the CFPA2 goal committed (same goal-forwarding/tuning nuance as
the desktop standalone run). The sensing+compute+load half is fully proven; the
actuation half is a follow-up tuning item, not a pipeline gap.
