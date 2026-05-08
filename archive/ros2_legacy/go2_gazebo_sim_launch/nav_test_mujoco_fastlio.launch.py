#!/usr/bin/env python3
"""MuJoCo nav test with Fast-LIO2 SLAM (replaces Cartographer).

Fast-LIO2 provides IMU-tight odometry with <5ms latency, eliminating
the scan-odom temporal misalignment that causes wall ghosting in
Cartographer mode. Everything else (FAR, CFPA2, etc.) is identical.

Nav backends (nav_backend:=):
  far   — CMU autonomy stack: terrain_analysis + far_planner + localPlanner + pathFollower (default)
  astar              — go2w_nav astar_nav_node (C++ 8-conn A* + pure-pursuit + footprint check)
  hybrid_astar       — go2w_nav hybrid_astar_nav_node (Hybrid A* + OMPL Reeds-Shepp + Ceres)
  hybrid             — alias for hybrid_astar
  nav2_hybrid_astar  — go2w_nav nav2_hybrid_astar_nav_node (nav2_smac_planner lib + Smoother)
  nav2               — alias for nav2_hybrid_astar
  nav2_mppi          — Full Nav2 stack: SmacPlannerHybrid + MPPI controller +
                       behavior_server + bt_navigator + lifecycle_manager,
                       plus cfpa2_to_nav2_bridge + path_relay + stuck_watchdog.
                       Pair with holonomic_profile:=se2_holonomic for the
                       lattice-planner forward/pivot profile shipped on real
                       Go2W 2026-05-02.

Modes:
  - Default: CFPA2 frontier exploration drives the robot autonomously.
  - Manual:  Pass explore:=false, then use RViz "2D Goal Pose" to send goals.

Usage:
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py nav_backend:=far
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py explore:=false   # manual goals only
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py gui:=false       # headless MuJoCo
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is
    launched in a namespace. Strip the outer key and return just the
    parameter dict so it can be merged into the launch params list.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    return data


def _launch_setup(context):
    # ═══════════════════════════════════════════════════════════════════
    # TF CHAIN — SINGLE-ROBOT SIM (Go2W or Go2, ns=robot)
    # ═══════════════════════════════════════════════════════════════════
    # NOTE: this is a 3rd TF pattern, distinct from both real (legacy
    # body-mount chain in real_single.launch.py) and mixed sim (adapter-
    # owned in nav_test_mujoco_fastlio_mixed.launch.py). Unification
    # candidate — see CLAUDE.md 2026-05-02 active-state entry.
    #
    #     world ──[static identity]──> map
    #                                    │
    #                         [static identity]
    #                                    │
    #                                    ▼
    #                                  odom
    #                                    │
    #                   [mujoco_odom_bridge dynamic, MuJoCo GT-driven]
    #                                    │
    #                                    ▼
    #                              base_link ──[RSP]──> URDF tree
    #
    # Edge owners:
    #   world → map           static_transform_publisher in this file (line ~302)
    #   map → odom            static_transform_publisher in this file (line ~302)
    #   odom → base_link      mujoco_odom_bridge (in single_go2w_mujoco_cfpa2.
    #                         launch.py base launch). Gated by the
    #                         odom_bridge_publish_tf flag passed into the
    #                         IncludeLaunchDescription at line ~159 — set TRUE
    #                         here on purpose (the OPPOSITE of the mixed-sim
    #                         contract, where mujoco_odom_bridge.publish_tf
    #                         must be False).
    #   base_link → URDF      robot_state_publisher
    #
    # When nav_backend=nav2_mppi (added 2026-05-02 PM): the new nav2_mppi
    # branch in this launch does NOT spawn fast_lio_tf_adapter — it relies
    # on the mujoco_odom_bridge edge above. SmacPlannerHybrid + MPPI work
    # fine against this chain; only the source of TF differs from mixed.
    #
    # Diagnostic:
    #   ros2 run tf2_ros tf2_echo map base_link \
    #       --ros-args -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static
    #   ros2 topic hz /mujoco_sim/mujoco_lidar_sensor/registered_scan
    #   ros2 topic hz /robot/Odometry        # Fast-LIO, validates SLAM separately
    #
    # Symptom of break: "Could not find a connection between 'odom' and
    # 'base_link'" → mujoco_odom_bridge not publishing TF. Check that
    # odom_bridge_publish_tf=true is propagating from this file's
    # IncludeLaunchDescription into single_go2w_mujoco_cfpa2.launch.py.
    # ═══════════════════════════════════════════════════════════════════
    use_sim_time = True
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    gui = _get(context, "gui")
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    # Bounded session + wall-checker toggles (for headless benchmark runs).
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_path = _get(context, "session_output_path").strip()
    enable_wall_checker = _as_bool(_get(context, "enable_wall_checker"))
    # Ground-truth observable area for coverage_ratio_of_scene. 96 m² is
    # the 12 m × 8 m inner room of demo1.xml.
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    # Velocity-aware safety supervisor (LiDAR-scan based velocity clamp).
    # When true, inserts scripts/velocity_safety_supervisor.py between
    # pathFollower and twist_bridge on the cmd_vel path. Real-robot safe.
    enable_velocity_supervisor = _as_bool(_get(context, "enable_velocity_supervisor"))
    # FAR's goal_point subscription topic. Default is CFPA2's direct output.
    # When TARE+mux is layered on top (nav_test_go2_tare.launch.py), we flip
    # this to /{ns}/way_point_coord_nav so FAR reads the muxed TARE/CFPA2
    # stream instead of the raw CFPA2 frontier.
    far_goal_topic = _get(context, "far_goal_topic").strip() or f"/{robot_ns}/way_point_coord"
    # FAR's way_point *output* topic. Default feeds localPlanner. The real-
    # TARE launch (nav_test_go2_tare_real.launch.py) bypasses FAR entirely —
    # TARE publishes straight to localPlanner's /{ns}/way_point input, and we
    # redirect FAR's own output to a dead sink so it can't collide.
    far_way_point_out = _get(context, "far_way_point_out").strip() or f"/{robot_ns}/way_point"
    # Go2W standing height ≈ 0.45 m (wheel axis); Go2 (non-W) stance ≈ 0.27 m.
    # The MID-360 is mounted 0.12 m above base; on Go2 that's z≈0.39 m vs Go2W
    # z≈0.57 m. Rays with ray-frame v_angle down to −7° combined with the 13°
    # forward-pitch site end up sweeping ground ~0.9–1.5 m ahead. Under fast
    # yaw/roll dynamics, pose jitter scatters those ground hits above the
    # 0.20 m filter, which then project as phantom walls. Raise the z-band
    # for pure Go2 so only points safely above chassis survive.
    has_wheels = _as_bool(_get(context, "has_wheels"))
    octo_point_cloud_min_z = 0.20 if has_wheels else 0.30
    octo_occupancy_min_z = 0.20 if has_wheels else 0.30

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    # vlm_pkg not needed — Cartographer config lives there, but Fast-LIO
    # config is in go2w_config/config/slam/pointlio_gazebo.yaml and is
    # loaded by the base launch.
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo1.xml")

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    nav_backend = _get(context, "nav_backend").strip().lower() or "far"
    # Back-compat aliases — rrt_star/mppi → astar (deleted 2026-04-24).
    if nav_backend == "rrt_star":
        nav_backend = "astar"
    # `hybrid`  → hybrid_astar (Ceres-smoothed Hybrid A*, our v0.1)
    # `nav2`    → nav2_hybrid_astar (B-route: nav2_smac_planner lib)
    if nav_backend == "hybrid":
        nav_backend = "hybrid_astar"
    if nav_backend == "nav2":
        nav_backend = "nav2_hybrid_astar"
    if nav_backend not in {"astar", "hybrid_astar", "nav2_hybrid_astar",
                           "nav2_mppi", "far"}:
        raise ValueError(
            f"nav_backend must be 'astar' | 'hybrid_astar' | 'nav2_hybrid_astar' | "
            f"'nav2_mppi' | 'far'; got '{nav_backend}'")
    # Optional Nav2 SE2-holonomic profile overlay. Only meaningful when
    # nav_backend=nav2_mppi. Mirrors the real-Go2W profile from 2026-05-02:
    #   off            → SmacPlannerHybrid + DiffDrive MPPI (default)
    #   se2_holonomic  → SmacPlannerLattice (diff primitives) + yaw-align/
    #                    forward MPPI, no lateral strafe
    holonomic_profile = (_get(context, "holonomic_profile").strip().lower() or "off")
    if holonomic_profile not in {"off", "se2_holonomic"}:
        raise ValueError(
            f"holonomic_profile must be 'off' | 'se2_holonomic'; got '{holonomic_profile}'")
    # Optional: load a real-robot Nav2 yaml in sim instead of the sim default.
    # Plain filename relative to go2w_config/config/nav/ (e.g.
    # 'nav2_go2_real.yaml' or 'nav2_go2w_real.yaml'). Empty = sim default.
    # Topic strings inside the real yaml use /robot/... which already match
    # this launch's default robot_namespace='robot', so no rewrites needed.
    nav2_yaml_override = _get(context, "nav2_yaml_override").strip()
    cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")

    actions = []

    # ── 1. Base platform: MuJoCo + CHAMP + sensors + perception ──
    base_launch = os.path.join(go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py")
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "true",
                "gui": gui,
                "rviz": "false",  # we launch our own RViz
                "cleanup_stale": "true",
                "enable_assets": "true",
                "enable_perception": "true",
                "enable_slam": "true",        # Fast-LIO2 provides SLAM
                "enable_control": "true",
                "enable_navigation": "false",  # we add our own below
                "use_fast_lio": "true",        # IMU-tight LIO, <5ms odom latency
                "odom_bridge_publish_tf": "true",   # Publish odom→base_link TF (50 Hz)
                "mujoco_model_path": mujoco_model_path,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
                "has_wheels": _get(context, "has_wheels"),
                "rl_policy": _get(context, "rl_policy"),
                "rl_use_champ_gains": _get(context, "rl_use_champ_gains"),
            }.items(),
        )
    )

    # ── 2. SLAM handled by base launch (Fast-LIO2) ──
    # Fast-LIO2 outputs 3D point cloud, NOT a 2D OccupancyGrid.
    # CFPA2 + RViz + astar_nav all need /robot/map (OccupancyGrid).
    # Use octomap_server to build a 3D voxel grid from the registered
    # scan, project to 2D, then binarize → /robot/map.
    slam_delay = 10.0

    slam_delay = 10.0

    # Octomap for /robot/map — runs for ALL nav backends (not just astar).
    # Fast-LIO has no occupancy grid; this generates one from the 3D scan.
    actions.append(
        TimerAction(
            period=slam_delay,
            actions=[
                Node(
                    package="octomap_server",
                    executable="octomap_server_node",
                    namespace=robot_ns,
                    name="octomap_map_gen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "resolution": 0.05,
                        "frame_id": "map",
                        "base_frame_id": "base_link",
                        "sensor_model.max_range": 8.0,
                        "sensor_model.hit": 0.8,
                        "sensor_model.miss": 0.35,
                        "sensor_model.min": 0.12,
                        "sensor_model.max": 0.97,
                        # Ground-return rejection: MID-360 at z≈0.57 m on Go2W
                        # (safe margin), z≈0.39 m on Go2 (tight). Rays at the
                        # lower vertical edge sweep ground, so the z-band
                        # filter is the last line of defense. Values are in
                        # the global (map) frame after TF. Walls go up to
                        # z=1.0; the upper max gives ceiling headroom.
                        "point_cloud_min_z": octo_point_cloud_min_z,
                        "point_cloud_max_z": 1.10,
                        "occupancy_min_z": octo_occupancy_min_z,
                        "occupancy_max_z": 1.00,
                        # filter_ground_plane runs RANSAC to find a ground
                        # plane. On Go2W it works — wheel-ground contact gives
                        # a clean flat plane. On pure Go2 with spherical feet
                        # the ground signature is sparser (only 4 point
                        # contacts vs 4 wheel disks), RANSAC fails every frame
                        # ("No ground plane found") and the projected map
                        # never updates → FAR sees stale map → robot STUCK.
                        # The z-band filter above already excludes the ground.
                        "filter_ground_plane": False,
                        # filter_speckles removes isolated single-voxel
                        # occupied cells — cheap speckle suppression for
                        # jitter-scattered ground hits that do slip above the
                        # z-filter under fast yaw dynamics on Go2.
                        "filter_speckles": True,
                        "compress_map": True,
                        "latch": True,
                        "publish_free_space": False,
                    }],
                    remappings=[
                        ("cloud_in", f"/{robot_ns}/registered_scan_reliable"),
                        ("projected_map", f"/{robot_ns}/map"),
                    ] + tf_remaps,
                    output="screen",
                ),
            ],
        )
    )

    # SC-PGO: Scan Context Pose Graph Optimization — adds loop closure
    # on top of Fast-LIO2. When the robot revisits an area, SC-PGO
    # detects the loop via scan context descriptors, runs ICP verification,
    # then optimizes the pose graph and publishes /corrected_odom.
    # slam_odom_relay already prefers /corrected_odom when available.
    # SC-PGO config paths are hard-coded to a previous developer's home
    # directory (/home/hz/...) and the package itself is optional — it
    # only adds loop-closure polish on top of Fast-LIO2, not a hard
    # dependency. If neither the package nor the config is present,
    # silently skip SC-PGO rather than crashing the launch.
    sc_pgo_config_candidates = [
        os.path.join(
            "/home/hz/COMP0225_LRC_stack/install/sc_pgo/share/sc_pgo/config",
            "sc_pgo_params.yaml",
        ),
        "/home/hz/COMP0225_LRC_stack/src/vendor/sc_pgo/config/sc_pgo_params.yaml",
        os.path.expanduser("~/COMP0225_LRC_stack/install/sc_pgo/share/sc_pgo/config/sc_pgo_params.yaml"),
    ]
    sc_pgo_config = next((p for p in sc_pgo_config_candidates if os.path.exists(p)), None)
    try:
        import ament_index_python.packages as _ament_pkg
        _ament_pkg.get_package_share_directory("sc_pgo")
        sc_pgo_available = True
    except Exception:
        sc_pgo_available = False

    if sc_pgo_config and sc_pgo_available:
        actions.append(
            TimerAction(
                period=slam_delay + 3.0,
                actions=[
                    Node(
                        package="sc_pgo",
                        executable="sc_pgo_node",
                        namespace=robot_ns,
                        name="sc_pgo",
                        parameters=[sc_pgo_config, {"use_sim_time": use_sim_time}],
                        remappings=[
                            # SC-PGO expects Fast-LIO topics
                            ("/aft_mapped_to_init", f"/{robot_ns}/Odometry"),
                            ("/cloud_registered", f"/{robot_ns}/cloud_registered_body"),
                            # SC-PGO output → slam_odom_relay picks this up
                            ("/corrected_odom", f"/{robot_ns}/corrected_odom"),
                            ("/corrected_path", f"/{robot_ns}/corrected_path"),
                            ("/corrected_cloud", f"/{robot_ns}/corrected_cloud"),
                            ("/corrected_map", f"/{robot_ns}/corrected_map"),
                        ] + tf_remaps,
                        output="screen",
                    ),
                ],
            )
        )
    else:
        reason = "package not installed" if not sc_pgo_available else "config file not found"
        actions.append(LogInfo(msg=(
            f"[nav_test_mujoco_fastlio] SC-PGO loop-closure skipped ({reason}). "
            f"Fast-LIO2 alone handles SLAM; SC-PGO is optional polish for long "
            f"trajectories. Install SC-PGO only if you specifically need it."
        )))

    # Static TFs to complete the tree. slam_odom_relay publishes odom
    # with frame_id="world", not "map". FAR + octomap need both frames.
    # world ≡ map (identity) for indoor SLAM without global localization.
    for parent, child in [("map", "odom"), ("world", "map")]:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_ns,
                name=f"{parent}_to_{child}_tf",
                arguments=[
                    "--frame-id", parent, "--child-frame-id", child,
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                remappings=[("/tf_static", f"/{robot_ns}/tf_static")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    # /{ns}/registered_scan_map was previously produced by a Python TF-based
    # pointcloud_frame_bridge with `transform_wait_sec=0.10` per cloud and a
    # 50 ms timer tick — measured total lag ~0.6 s vs. Fast-LIO's state
    # estimate. terrain_analysis filtered those stale points against fresh
    # odom → phantom voxels at the robot's former position.
    #
    # The replacement below applies a single constant rotation+translation
    # (the same offset slam_odom_relay puts on odom to go camera_init →
    # world) to Fast-LIO's already-world-aligned /cloud_registered, with
    # zero artificial wait and numpy-vectorized math.
    if nav_backend == "far":
        actions.append(
            TimerAction(
                period=slam_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3", "-u",
                            os.path.join(_ws_root, "scripts/runtime/cloud_world_offset_bridge.py"),
                            "--ros-args",
                            "-r", f"__ns:=/{robot_ns}",
                            "-p", "use_sim_time:=true",
                            "-p", f"cloud_input_topic:=/{robot_ns}/cloud_registered_camera_init",
                            "-p", f"cloud_output_topic:=/{robot_ns}/registered_scan_map",
                            "-p", f"raw_odom_topic:=/{robot_ns}/Odometry",
                            "-p", f"world_odom_topic:=/{robot_ns}/odom/nav",
                            "-p", "output_frame:=map",
                        ],
                        name="cloud_world_offset_bridge",
                        output="screen",
                    ),
                ],
            )
        )

    # ── 3. CFPA2 frontier exploration (optional) ──
    nav_delay = slam_delay + 3.0

    if explore:
        actions.append(
            TimerAction(
                period=nav_delay,
                actions=[
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_single_robot_node",
                        name="cfpa2_single_robot",
                        parameters=[
                            cfpa2_config_path,
                            {
                                "use_sim_time": use_sim_time,
                                "robot_namespace": robot_ns,
                                "namespaces": [robot_ns],
                                "goal_topic_suffix": "/way_point_coord",
                                "marker_frame_override": "map",
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── 4. Navigation backend ──
    if nav_backend == "nav2_mppi":
        # Full Nav2 stack: SmacPlannerHybrid (or Lattice via SE2 overlay) +
        # MPPI controller + behavior_server + bt_navigator + lifecycle_manager.
        # Mirrors the heterogeneous mixed-demo setup but for one robot.
        from launch_ros.actions import PushRosNamespace
        from launch.actions import GroupAction
        from nav2_common.launch import RewrittenYaml

        # Per-platform base yaml (same files used by the real stack).
        # Override (nav2_yaml_override) lets us drop a real-robot yaml in
        # for sim-vs-real unification experiments. RewrittenYaml below flips
        # use_sim_time → true regardless of what the file says, so a real
        # yaml (use_sim_time: false) loads cleanly.
        if nav2_yaml_override:
            _nav2_yaml_filename = nav2_yaml_override
        else:
            _nav2_yaml_filename = (
                "nav2_go2w_full_stack.yaml" if has_wheels
                else "nav2_go2_full_stack.yaml"
            )
        nav2_yaml = os.path.join(
            go2w_config_pkg, "config", "nav", _nav2_yaml_filename
        )
        if not os.path.exists(nav2_yaml):
            raise FileNotFoundError(
                f"Nav2 yaml not found: {nav2_yaml}. Check spelling of "
                f"nav2_yaml_override (must be a filename under "
                f"go2w_config/config/nav/, not a path)."
            )
        # The full-stack yamls were authored for dual sim:
        #   nav2_go2w_full_stack.yaml  → /robot_a/* topics
        #   nav2_go2_full_stack.yaml   → /robot_b/* topics, robot_base_frame=b_base_link
        # Real yamls use /robot/* and base_link.
        # In single sim, robot_ns defaults to "robot" and TF publishes plain
        # base_link (no b_ prefix) regardless of platform — so without
        # rewriting both topic prefixes AND robot_base_frame, controller_server
        # never finishes activating (TF lookup for b_base_link → odom fails),
        # bt_navigator stays inactive, and goals from cfpa2_to_nav2_bridge are
        # silently dropped (root cause of the 2026-05-02 "robot stuck at spawn"
        # bug — single Go2 sim nav2_mppi was untested before).
        # Strategy: pre-process the yaml text to rewrite /robot_[ab]/ →
        # /{robot_ns}/, write to a temp file, then let RewrittenYaml flip
        # use_sim_time and force base_link for any single-sim Go2 instance.
        import re as _re
        import tempfile as _tempfile
        with open(nav2_yaml) as _f:
            _yaml_text = _f.read()
        _yaml_text = _re.sub(r"/robot_[ab]/", f"/{robot_ns}/", _yaml_text)
        _tmp_yaml = _tempfile.NamedTemporaryFile(
            mode="w", suffix=f"_{robot_ns}_nav2.yaml", delete=False
        )
        _tmp_yaml.write(_yaml_text)
        _tmp_yaml.close()
        rewritten_nav2 = RewrittenYaml(
            source_file=_tmp_yaml.name,
            root_key=robot_ns,
            param_rewrites={
                "use_sim_time": str(use_sim_time).lower(),
                # Single sim publishes plain base_link; force it across all
                # nodes regardless of what the source yaml says (the dual-sim
                # Go2 yaml says b_base_link, real yaml already says base_link).
                "robot_base_frame": "base_link",
            },
            convert_types=True,
        )

        # Optional SE2-holonomic overlay — same file as mixed sim.
        nav2_params = [rewritten_nav2]
        if holonomic_profile == "se2_holonomic":
            overlay_path = os.path.join(
                go2w_config_pkg, "config", "nav",
                "nav2_se2_holonomic_overlay_sim.yaml",
            )
            overlay_rewritten = RewrittenYaml(
                source_file=overlay_path,
                root_key=robot_ns,
                param_rewrites={"use_sim_time": str(use_sim_time).lower()},
                convert_types=True,
            )
            nav2_params.append(overlay_rewritten)

        # cmd_vel routing in single sim:
        #   Go2W (has_wheels=true): go2w_hybrid_cmd_router subscribes to plain
        #     /<ns>/cmd_vel and splits to wheel + cmd_vel_legged. Default
        #     Nav2 publish destination is correct → no remap.
        #   Go2 (has_wheels=false): no router; CHAMP's quadruped_controller_node
        #     listens directly on /<ns>/cmd_vel_legged (the twist_bridge target,
        #     used historically by default_nav/astar via cmd_vel_stamped). Nav2
        #     publishes Twist on relative cmd_vel → /<ns>/cmd_vel which has zero
        #     subscribers, so the robot stands still even with a healthy MPPI.
        #     Remap controller_server + behavior_server cmd_vel to cmd_vel_legged
        #     so CHAMP receives Nav2's commands. Discovered 2026-05-02 PM during
        #     the real-yaml unification experiment — single Go2 sim nav2_mppi
        #     was committed but never end-to-end tested.
        nav2_cmd_remap = [] if has_wheels else [("cmd_vel", "cmd_vel_legged")]

        nav2_inner_nodes = [
            PushRosNamespace(robot_ns),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server",
                parameters=nav2_params,
                remappings=tf_remaps + nav2_cmd_remap, output="screen",
            ),
            Node(
                package="nav2_planner", executable="planner_server",
                name="planner_server",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server",
                parameters=nav2_params,
                remappings=tf_remaps + nav2_cmd_remap, output="screen",
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator",
                name="bt_navigator",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=nav2_params,
                output="screen",
            ),
        ]

        # _ws_root is module-level (declared near the imports) so every
        # ExecuteProcess in this file can pick up the auto-resolved repo
        # root without redefining it.

        # CFPA2 → Nav2 goal bridge: way_point_coord (PointStamped) → goal_pose
        # (PoseStamped, BEST_EFFORT to match bt_navigator). When explore=false
        # the bridge is a no-op (CFPA2 isn't running) and RViz "2D Goal Pose"
        # is the goal source — but the bridge is harmless to leave running.
        bridge_node = ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(_ws_root, "scripts/runtime/cfpa2_to_nav2_bridge.py"),
                "--ros-args",
                "-p", f"namespace:={robot_ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                "-p", "waypoint_topic:=way_point_coord",
            ],
            name=f"cfpa2_to_nav2_bridge_{robot_ns}",
            output="screen",
        )

        # Nav2 /plan → /planned_path so existing RViz config catches it.
        path_relay_node = ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(_ws_root, "scripts/runtime/path_relay.py"),
                "--ros-args",
                "-p", f"namespace:={robot_ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"path_relay_{robot_ns}",
            output="screen",
        )

        # stuck_watchdog: see CLAUDE.md golden rule #18 — MPPI rarely reports
        # failure in pivot-stuck cases (emits v≈ω≈0 with status=happy), so
        # Nav2's BT recovery never fires. This watchdog detects 10 s of
        # no-motion under an active goal and fires a BackUp action.
        stuck_watchdog_node = ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(_ws_root, "scripts/runtime/stuck_watchdog.py"),
                "--ros-args",
                "-p", f"namespace:={robot_ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"stuck_watchdog_{robot_ns}",
            output="screen",
        )

        # hybrid_cmd_router: only Go2W needs it (splits cmd_vel into
        # wheel + legged components based on curvature). Go2 (no wheels)
        # has CHAMP listening to /<ns>/cmd_vel directly.
        if has_wheels:
            router_node = Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=robot_ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    os.path.join(go2w_config_pkg, "config", "control",
                                 "go2w_hybrid_motion.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_command_topic":
                            f"/mujoco_sim/{robot_ns}_wheel_velocity_controller/commands",
                    },
                ],
                output="screen",
            )
        else:
            router_node = None

        _nav2_actions = [
            GroupAction(actions=nav2_inner_nodes),
            bridge_node,
            path_relay_node,
            stuck_watchdog_node,
        ]
        if router_node is not None:
            _nav2_actions.append(router_node)
        actions.append(
            TimerAction(period=nav_delay, actions=_nav2_actions)
        )

    elif nav_backend in ("astar", "hybrid_astar", "nav2_hybrid_astar"):
        # All three share the same I/O contract and supporting infra
        # (octomap_server + map_merger). They differ only in the nav
        # executable + yaml config:
        #   astar              → astar_nav_node            (8-conn A* + Option B)
        #   hybrid_astar       → hybrid_astar_nav_node     (our v0.1: OMPL RS + Ceres)
        #   nav2_hybrid_astar  → nav2_hybrid_astar_nav_node (B-route: nav2_smac lib)
        if nav_backend == "astar":
            nav_config_path = os.path.join(
                go2w_config_pkg, "config", "nav", "astar_nav_go2w.yaml")
            nav_executable = "astar_nav_node"
            nav_node_name  = "astar_nav"
        elif nav_backend == "hybrid_astar":
            nav_config_path = os.path.join(
                go2w_config_pkg, "config", "nav", "hybrid_astar_nav_go2w.yaml")
            nav_executable = "hybrid_astar_nav_node"
            nav_node_name  = "hybrid_astar_nav"
        else:  # nav2_hybrid_astar
            nav_config_path = os.path.join(
                go2w_config_pkg, "config", "nav", "nav2_hybrid_astar_nav_go2w.yaml")
            nav_executable = "nav2_hybrid_astar_nav_node"
            nav_node_name  = "nav2_hybrid_astar_nav"
        nav_remappings = [
            ("/way_point", f"/{robot_ns}/way_point_coord"),
            ("/odom/ground_truth", f"/{robot_ns}/odom/nav"),
            ("/scan", f"/{robot_ns}/scan_3d"),
            ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{robot_ns}/nav_status"),
            ("/planned_path", f"/{robot_ns}/planned_path"),
            ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
            ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
            ("/robot_pose_marker", f"/{robot_ns}/robot_pose_marker"),
        ]

        # octomap_server: 3D voxel grid from the reliable registered scan
        # projected to 2D. In Fast-LIO mode it IS the /{ns}/map source
        # (no Cartographer to merge with), so the map_merger below is a
        # pass-through and astar reads /{ns}/map directly.
        octomap_node = Node(
            package="octomap_server",
            executable="octomap_server_node",
            namespace=robot_ns,
            name="octomap_server",
            parameters=[{
                "use_sim_time": use_sim_time,
                "resolution": 0.05,
                "frame_id": "map",
                "base_frame_id": "base_link",
                "sensor_model.max_range": 6.0,
                "sensor_model.hit": 0.8,
                "sensor_model.miss": 0.35,
                "sensor_model.min": 0.12,
                "sensor_model.max": 0.97,
                "point_cloud_min_z": 0.05,
                "point_cloud_max_z": 1.10,
                "occupancy_min_z": 0.05,
                "occupancy_max_z": 1.00,
                "filter_ground_plane": False,
                "incremental_2D_projection": False,
                "filter_speckles": False,
                "compress_map": True,
                # latch=True → TRANSIENT_LOCAL, matches CFPA2 map sub QoS.
                "latch": True,
                "publish_free_space": False,
            }],
            remappings=[
                ("cloud_in", f"/{robot_ns}/registered_scan_reliable"),
                # Publish directly as /robot/map in Fast-LIO mode.
                ("projected_map", f"/{robot_ns}/map"),
            ] + tf_remaps,
            output="screen",
        )

        # map_merger pass-through kept for topic-name parity with the
        # Cartographer launch variant; same source on primary + secondary.
        map_merger_node = Node(
            package="go2w_perception",
            executable="map_merger.py",
            namespace=robot_ns,
            name="map_merger",
            parameters=[{
                "use_sim_time": use_sim_time,
                "primary_topic": f"/{robot_ns}/map",
                "secondary_topic": f"/{robot_ns}/map",
                "output_topic": f"/{robot_ns}/map_merged",
                "secondary_occupied_thresh": 50,
                "publish_rate_hz": 4.0,
            }],
            output="screen",
        )

        astar_nav_node = Node(
            package="go2w_nav",
            executable=nav_executable,
            namespace=robot_ns,
            name=nav_node_name,
            parameters=[
                nav_config_path,
                {"use_sim_time": use_sim_time},
                {
                    "map_frame": "map",
                    "map_topic": f"/{robot_ns}/map_merged",
                    "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                    "stop_topic": f"/{robot_ns}/stop",
                },
            ],
            remappings=nav_remappings + tf_remaps,
            output="screen",
        )

        actions.append(
            TimerAction(
                period=nav_delay,
                actions=[octomap_node, map_merger_node, astar_nav_node],
            )
        )
    else:  # nav_backend == "far"
        far_scan_topic = f"/{robot_ns}/registered_scan_map"
        far_odom_topic = f"/{robot_ns}/odom/nav"
        # Iter 6 (2026-04-15): 0.2 m/s is the proven-safe default at the
        # position-based checks. A 0.4 m/s ceiling only makes sense when
        # the velocity-aware supervisor is enabled (enable_velocity_supervisor
        # launch arg), which caps cmd_vel to v_cap = sqrt(2·a·(d_nearest-
        # d_safe)) based on live scan clearance. Without the supervisor,
        # stay at 0.2 — position checks + twoWayDrive + checkRotObstacle
        # give 7/10 PASS at 120s with two failure modes (corner-wedge +
        # yaw-drift stall).
        far_max_speed_override = _get(context, "far_max_speed").strip()
        if far_max_speed_override:
            far_max_speed = float(far_max_speed_override)
        else:
            far_max_speed = 0.4 if enable_velocity_supervisor else 0.2

        # Reverse drive — controls twoWayDrive in both localPlanner and
        # pathFollower. On Go2W wheels can spin backward trivially; on pure
        # Go2 CHAMP's "go2" preset lacks a validated reverse-walking gait,
        # so FAR commanding REVERSE leaves the robot stuck (v2 smoke test
        # 2026-04-17). Default: inherit has_wheels — Go2W=true, Go2=false.
        two_way_drive_override = _get(context, "two_way_drive").strip().lower()
        if two_way_drive_override in ("true", "false"):
            two_way_drive = two_way_drive_override == "true"
        else:
            two_way_drive = has_wheels  # inherit: Go2W=reverse-ok, Go2=no-reverse

        # pathFollower output topic: canonical cmd_vel_stamped when no
        # supervisor, or a "_raw" sibling that the supervisor consumes.
        if enable_velocity_supervisor:
            pf_cmd_out_topic = f"/{robot_ns}/cmd_vel_stamped_raw"
        else:
            pf_cmd_out_topic = f"/{robot_ns}/cmd_vel_stamped"
        local_planner_pkg = get_package_share_directory("local_planner")

        far_nodes = [
            # sensor_scan_generation: sync odom+cloud (WARN to reduce noise)
            Node(
                package="sensor_scan_generation",
                executable="sensorScanGeneration",
                namespace=robot_ns,
                name="sensor_scan_generation",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/state_estimation_at_scan", f"/{robot_ns}/state_estimation_at_scan"),
                    ("/sensor_scan", f"/{robot_ns}/sensor_scan"),
                ] + tf_remaps,
                output="screen",
            ),
            # terrain_analysis: local terrain voxel map (WARN to reduce noise)
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                namespace=robot_ns,
                name="terrain_analysis",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "maxRelZ": 0.8,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/map_clearing", f"/{robot_ns}/map_clearing"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                ],
                output="screen",
            ),
            # terrain_analysis_ext: extended range terrain
            Node(
                package="terrain_analysis_ext",
                executable="terrainAnalysisExt",
                namespace=robot_ns,
                name="terrain_analysis_ext",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "maxRelZ": 0.8,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/cloud_clearing", f"/{robot_ns}/cloud_clearing"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                    ("/terrain_map_ext", f"/{robot_ns}/terrain_map_ext"),
                ],
                output="screen",
            ),
            # far_planner: V-graph global route planner
            Node(
                package="far_planner",
                executable="far_planner",
                namespace=robot_ns,
                name="far_planner",
                parameters=[
                    # FAR upstream defaults
                    _load_yaml_params(os.path.join(
                        get_package_share_directory("far_planner"), "config", "default.yaml"
                    )),
                    # Tuning overrides — edit this YAML for rapid testing
                    # (no rebuild needed, just re-launch):
                    os.path.join(go2w_config_pkg, "config", "nav", "far_planner_tuning.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "graph_msger/robot_id": 0,
                    },
                ],
                remappings=[
                    ("/odom_world", far_odom_topic),
                    ("/terrain_cloud", f"/{robot_ns}/terrain_map_ext"),
                    ("/scan_cloud", f"/{robot_ns}/terrain_map"),
                    ("/terrain_local_cloud", far_scan_topic),
                    ("/goal_point", far_goal_topic),
                    ("/way_point", far_way_point_out),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/navigation_boundary", f"/{robot_ns}/navigation_boundary"),
                    ("/runtime", f"/{robot_ns}/far_runtime"),
                    ("/planning_time", f"/{robot_ns}/far_planning_time"),
                    ("/robot_vgraph", f"/{robot_ns}/robot_vgraph"),
                    ("/decoded_vgraph", f"/{robot_ns}/decoded_vgraph"),
                ] + tf_remaps,
                output="screen",
            ),
            # localPlanner: kinematically-feasible path primitives
            Node(
                package="local_planner",
                executable="localPlanner",
                namespace=robot_ns,
                name="localPlanner",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "pathFolder": os.path.join(local_planner_pkg, "paths"),
                    "vehicleLength": 0.70,
                    "vehicleWidth": 0.40,
                    "sensorOffsetX": 0.0,
                    "sensorOffsetY": 0.0,
                    "twoWayDrive": two_way_drive,
                    "laserVoxelSize": 0.05,
                    "terrainVoxelSize": 0.2,
                    "useTerrainAnalysis": True,
                    "checkObstacle": True,
                    # checkRotObstacle=False on Go2 lets FAR rotate-in-place
                    # without requiring a clear rotation primitive. With
                    # two_way_drive=false the robot has no reverse primitive,
                    # so when its rotation primitives also all get rejected
                    # by the obstacle check, pathFollower decays cmd_vel to 0
                    # and the whole stack sits idle. Go2W keeps the stricter
                    # check (wheels can reverse out of trouble).
                    "checkRotObstacle": has_wheels,
                    "adjacentRange": 4.0,
                    "obstacleHeightThre": 0.20,
                    "groundHeightThre": 0.1,
                    "costHeightThre": 0.1,
                    "costScore": 0.02,
                    "useCost": False,
                    "pointPerPathThre": 2,
                    "minRelZ": -0.5,
                    "maxRelZ": 1.2,
                    "maxSpeed": far_max_speed,
                    # Iter 7: forward-bias — strongly prefer forward primitives.
                    "dirWeight": 0.5,
                    "dirThre": 90.0,
                    "dirToVehicle": False,
                    "pathScale": 1.0,
                    "minPathScale": 0.75,
                    "pathScaleStep": 0.25,
                    "pathScaleBySpeed": False,
                    "minPathRange": 1.0,
                    "pathRangeStep": 0.5,
                    "pathRangeBySpeed": False,
                    "pathCropByGoal": True,
                    "autonomyMode": True,
                    "autonomySpeed": far_max_speed,
                    "joyToSpeedDelay": 2.0,
                    "joyToCheckObstacleDelay": 5.0,
                    "goalClearRange": 0.5,
                    "goalX": 0.0,
                    "goalY": 0.0,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/way_point", f"/{robot_ns}/way_point"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                    ("/overall_map", f"/{robot_ns}/terrain_map"),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/path", f"/{robot_ns}/local_path"),
                    ("/freePaths", f"/{robot_ns}/free_paths"),
                ],
                output="screen",
            ),
            # pathFollower: pure-pursuit path tracking → cmd_vel
            Node(
                package="local_planner",
                executable="pathFollower",
                namespace=robot_ns,
                name="pathFollower",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "sensorOffsetX": 0.0,
                    "sensorOffsetY": 0.0,
                    "pubSkipNum": 1,
                    "twoWayDrive": two_way_drive,
                    "lookAheadDis": 0.8,
                    "yawRateGain": 1.0,
                    "stopYawRateGain": 0.8,
                    "maxYawRate": 45.0,
                    "maxSpeed": far_max_speed,
                    "maxAccel": 2.0,
                    "switchTimeThre": 1.0,
                    # Iter 7: forward-bias — higher threshold before reverse.
                    "dirDiffThre": 1.2,
                    "omniDirDiffThre": 1.5,
                    "noRotSpeed": 10.0,
                    "stopDisThre": 0.25,
                    "slowDwnDisThre": 0.50,
                    "useInclRateToSlow": False,
                    "inclRateThre": 120.0,
                    "slowRate1": 0.25,
                    "slowRate2": 0.5,
                    "slowTime1": 2.0,
                    "slowTime2": 2.0,
                    "useInclToStop": False,
                    "inclThre": 45.0,
                    "stopTime": 5.0,
                    "noRotAtStop": False,
                    "noRotAtGoal": True,
                    "autonomyMode": True,
                    "autonomySpeed": far_max_speed,
                    "joyToSpeedDelay": 2.0,
                    "goalCloseDis": 0.6,
                    "is_real_robot": False,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/path", f"/{robot_ns}/local_path"),
                    ("/cmd_vel", pf_cmd_out_topic),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/speed", f"/{robot_ns}/speed"),
                    ("/stop", f"/{robot_ns}/stop"),
                ],
                output="screen",
            ),
            # Static TFs for CMU local planner convention
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_ns,
                name="far_vehicle_tf",
                arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
                remappings=tf_remaps,
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_ns,
                name="far_camera_tf",
                arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
                remappings=tf_remaps,
                output="screen",
            ),
        ]

        actions.append(TimerAction(period=nav_delay, actions=far_nodes))

        # ── 4b. Velocity-aware safety supervisor (optional) ──
        # Caps commanded velocity by live scan-based nearest-obstacle
        # clearance. Only wired into the FAR path because pathFollower is
        # the cmd_vel source there; astar branch would need a separate
        # hookup on astar_nav_node's output.
        if enable_velocity_supervisor:
            supervisor_script = os.path.join(_ws_root, "scripts/runtime/velocity_safety_supervisor.py")
            supervisor_proc = ExecuteProcess(
                cmd=[
                    "python3", "-u", supervisor_script,
                    "--ros-args",
                    "-p", f"scan_topic:=/{robot_ns}/scan_3d",
                    "-p", f"cmd_in_topic:=/{robot_ns}/cmd_vel_stamped_raw",
                    "-p", f"cmd_out_topic:=/{robot_ns}/cmd_vel_stamped",
                    "-p", f"max_linear_speed_m_s:={far_max_speed}",
                    "-p", "max_decel_m_s2:=2.0",
                    "-p", "safety_margin_m:=0.10",
                    "-p", "forward_arc_half_rad:=1.047",
                    "-p", "publish_rate_hz:=50.0",
                ],
                name="velocity_safety_supervisor",
                output="screen",
            )
            actions.append(
                TimerAction(period=nav_delay, actions=[supervisor_proc])
            )

    # ── 5. RViz goal relay (always on — click "2D Goal Pose" to send manual goals) ──
    actions.append(
        TimerAction(
            period=nav_delay,
            actions=[
                Node(
                    package="go2w_nav",
                    executable="rviz_goal_relay.py",
                    namespace=robot_ns,
                    name="rviz_goal_relay",
                    parameters=[
                        {"output_topic": f"/{robot_ns}/way_point_coord"},
                    ],
                    output="screen",
                ),
            ],
        )
    )

    # ── 5b. FAR debug monitor ──
    # Integrated into launch — prints 1-line/sec summary of FAR I/O
    # with color-coded STUCK/REVERSE/OSCILLATE/CONTACT warnings.
    # Silences non-FAR nodes to keep the terminal readable.
    far_debug_script = os.path.join(_ws_root, "scripts/debug/far_debug_monitor.py")
    actions.append(
        TimerAction(
            period=nav_delay + 5.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", "-u", far_debug_script],
                    name="far_debug_monitor",
                    output="screen",
                ),
            ],
        )
    )

    # ── 6. Wall / tip-over fail checker (optional — terminal) ──
    # Runs scripts/far_wall_checker.py as a launch process. On robot-body
    # contact with wall_/divider_ geoms OR tip-over (|roll|, |pitch| > 45°),
    # the checker prints a FAIL banner and exits 1. The OnProcessExit event
    # handler then shuts down the entire launch so the failure is terminal.
    # Disabled via `enable_wall_checker:=false` for benchmark runs that
    # want to measure contact count without the launch dying on first hit.
    if enable_wall_checker:
        wall_checker_script = os.path.join(_ws_root, "scripts/runtime/far_wall_checker.py")
        wall_checker_proc = ExecuteProcess(
            cmd=["python3", "-u", wall_checker_script],
            name="far_wall_checker",
            output="screen",
        )
        actions.append(
            TimerAction(
                period=nav_delay + 3.0,  # wait until standup + nav are done
                actions=[wall_checker_proc],
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wall_checker_proc,
                    on_exit=[
                        LogInfo(
                            msg="far_wall_checker exited — shutting down launch "
                            "(robot hit a wall or tipped over)"
                        ),
                        Shutdown(reason="far_wall_checker detected failure"),
                    ],
                )
            )
        )

    # ── 6b. Bounded session reporter (optional — graceful exit on timeout) ──
    # Runs scripts/session_reporter.py for `session_duration_sec` seconds.
    # Subscribes to /<ns>/map, /<ns>/odom/nav, /mujoco/contacts and emits a
    # JSON report to `session_output_path` on final tick (or SIGTERM). When
    # it exits cleanly, the OnProcessExit handler shuts the whole launch
    # down so headless benchmark runs reliably terminate at the set bound.
    if session_duration_sec > 0.0:
        if not session_output_path:
            session_output_path = "/tmp/session_reports/latest.json"
        session_script = os.path.join(_ws_root, "scripts/bench/session_reporter.py")
        session_proc = ExecuteProcess(
            cmd=[
                "python3", "-u", session_script,
                "--duration", str(session_duration_sec),
                "--namespace", robot_ns,
                "--output", session_output_path,
                "--scene-area-m2", str(scene_area_m2),
            ],
            name="session_reporter",
            output="screen",
        )
        actions.append(
            TimerAction(
                period=nav_delay + 3.0,
                actions=[session_proc],
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=session_proc,
                    on_exit=[
                        LogInfo(
                            msg="session_reporter exited — shutting down launch "
                            "(bounded session complete)"
                        ),
                        Shutdown(reason="session_reporter session complete"),
                    ],
                )
            )
        )

    # ── 7. RViz2 ──
    if rviz:
        rviz_config = os.path.join(go2_gazebo_pkg, "rviz", "nav_test.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_nav_test",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        remappings=tf_remaps,
                        output="screen",
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    default_scene = os.path.join(go2_gazebo_pkg, "mujoco", "demo1.xml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true",
                              description="Enable CFPA2 autonomous frontier exploration"),
        DeclareLaunchArgument(
            "nav_backend", default_value="nav2_mppi",
            description=(
                "Nav backend (DEFAULT 'nav2_mppi'): SmacPlannerHybrid (or "
                "SmacPlannerLattice when holonomic_profile=se2_holonomic, "
                "the default) + MPPI + behavior_server + stuck_watchdog. "
                "Other choices: 'far' (CMU stack), 'astar' (C++ A* + "
                "oriented footprint check), 'hybrid_astar', "
                "'nav2_hybrid_astar'."
            ),
        ),
        DeclareLaunchArgument(
            "holonomic_profile", default_value="se2_holonomic",
            description=(
                "Nav2 SE2-holonomic overlay (only takes effect with "
                "nav_backend=nav2_mppi). DEFAULT 'se2_holonomic' = "
                "SmacPlannerLattice (diff primitives) + yaw-align/forward "
                "MPPI, no lateral strafe — mirrors the real-Go2W profile "
                "shipped 2026-05-02; fits the legged + wheeled-legged "
                "kinematic model best. Pass holonomic_profile:=off to "
                "fall back to SmacPlannerHybrid + DiffDrive MPPI baseline."
            ),
        ),
        DeclareLaunchArgument(
            "nav2_yaml_override", default_value="",
            description=(
                "Plain filename under go2w_config/config/nav/ to load as "
                "the Nav2 base yaml instead of the sim default. Used for "
                "sim/real unification experiments — e.g. "
                "'nav2_go2_real.yaml' or 'nav2_go2w_real.yaml'. "
                "RewrittenYaml flips use_sim_time → true automatically. "
                "Sim's default robot_namespace='robot' matches the real "
                "yaml's /robot/* topic prefixes, so no remap is needed."
            ),
        ),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="4.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("session_duration_sec", default_value="0",
                              description="If > 0, run bounded session reporter "
                              "and shut launch down after N seconds"),
        DeclareLaunchArgument("session_output_path",
                              default_value="/tmp/session_reports/latest.json",
                              description="JSON output path for the session reporter"),
        DeclareLaunchArgument("scene_area_m2", default_value="96.0",
                              description="Sim ground-truth observable area (m²) "
                              "used as denominator for coverage_ratio_of_scene. "
                              "Default 96 for vlm_exploration_scene_no_artifacts."),
        DeclareLaunchArgument("enable_wall_checker", default_value="false",
                              description="Test mode: crash-stop on first wall contact. "
                              "Default false (benchmark mode — session reporter "
                              "captures all contacts without killing the launch). "
                              "Use enable_wall_checker:=true when developing to "
                              "get instant feedback on wall hits."),
        DeclareLaunchArgument("enable_velocity_supervisor", default_value="false",
                              description="Enable LiDAR-scan velocity-aware safety "
                              "supervisor between pathFollower and twist_bridge; "
                              "allows far_max_speed=0.4 to be safe"),
        DeclareLaunchArgument("far_max_speed", default_value="",
                              description="Override localPlanner/pathFollower maxSpeed "
                              "(m/s). Empty = use default (0.2 without supervisor, "
                              "0.4 with). Untested >0.4 — tune at your own risk."),
        DeclareLaunchArgument("has_wheels", default_value="true",
                              description="Set to false for pure Go2 (non-W) — skips "
                              "the wheel_velocity_controller spawn and go2w_hybrid_"
                              "cmd_router. Pair with a Go2 MJCF (e.g. demo1_go2.xml)."),
        DeclareLaunchArgument("two_way_drive", default_value="",
                              description="Override localPlanner/pathFollower "
                              "twoWayDrive (reverse primitives + fwd/rev switching). "
                              "Empty = inherit from has_wheels (Go2W=true, Go2=false). "
                              "Pass 'true'/'false' to force."),
        DeclareLaunchArgument("rl_policy", default_value="false",
                              description="Run the Isaac-Lab ONNX flat policy in "
                              "place of CHAMP (requires has_wheels:=false). See "
                              "single_go2w_mujoco_cfpa2.launch.py for full notes."),
        DeclareLaunchArgument("rl_use_champ_gains", default_value="false",
                              description="With rl_policy:=true, use CHAMP's stiff "
                              "kp=100/kd=1.0 PD gains instead of the training kp=20/kd=0.5. "
                              "Allows the pre-RL stand-up trajectory to hold the robot "
                              "upright at the cost of 5× policy torque overshoot."),
        DeclareLaunchArgument("far_goal_topic", default_value="",
                              description="Override topic FAR subscribes to for the "
                              "global goal (normally CFPA2 → /{ns}/way_point_coord). "
                              "Empty = default. The TARE wrapper "
                              "(nav_test_go2_tare.launch.py) sets this to "
                              "/{ns}/way_point_coord_nav so FAR reads the TARE/CFPA2 "
                              "mux output instead of the raw CFPA2 frontier."),
        DeclareLaunchArgument("far_way_point_out", default_value="",
                              description="Override topic FAR publishes its local "
                              "waypoint to (normally /{ns}/way_point → localPlanner). "
                              "Empty = default. The real-TARE launch "
                              "(nav_test_go2_tare_real.launch.py) sets this to a dead "
                              "topic so TARE can own /{ns}/way_point without a "
                              "publisher collision."),
        OpaqueFunction(function=_launch_setup),
    ])
