#!/usr/bin/env python3
"""Single-robot A*-planner nav benchmark launch.

One robot (Go2W or Go2), one scene, `astar_nav_node` as the only local/global
planner. No FAR, no terrain_analysis, no localPlanner, no pathFollower, no
MPPI. A* on the octomap occupancy grid, pure-pursuit on a resampled path,
curvature-aware speed shaping, and automatic wheel ↔ leg mode bias via the
downstream hybrid_cmd_router.

Core motion design (Go2W): at straight stretches, v climbs above the hybrid
router's wheel threshold → wheels drive; in tight corners, v drops and |ω|
rises → router selects legged → CHAMP pivots in place. Near goal, brake to
goal_slow_floor to avoid overshoot.

Stack:

    MuJoCo plugin  ──▶  mujoco_odom_bridge  ──▶  /robot/odom/ground_truth
                   └──▶ LiDAR /mujoco_sim/...   /robot/imu/data
                                 │
                                 ▼
                        qos_bridge + pointcloud_adapter
                                 │
                                 ▼
                        Fast-LIO2  ──▶  /robot/Odometry
                           │              ▼
                           ▼         slam_odom_relay
                        octomap_server  ──▶  /robot/odom/nav
                           │
                   ┌───────┴────────┐
                   ▼                ▼
              /robot/map      pointcloud_to_laserscan
                   │                │
                   │                ▼
                   │         /robot/scan_3d
                   │                │
                   └────────────────┴────▶  astar_nav_node
                                             (also: /robot/way_point_coord from CFPA2)
                                             │
                                             ▼
                                    /robot/cmd_vel_stamped
                                             │ twist_bridge
                                             ▼
                                    /robot/cmd_vel
                                             │
                         ┌───────────────────┴──────────────────┐
                         ▼                                       ▼
                  Go2W: hybrid_cmd_router             Go2: cmd_vel (direct to CHAMP)
                         │
                 ┌───────┴────────────┐
                 ▼                    ▼
         cmd_vel_legged ─▶ CHAMP    wheel_velocity_controller

Args:
    robot:=go2w | go2                  (default go2w)
    scene:=demo1 | demo3               (default demo1)
    gui:=true|false                    (default false)
    rviz:=true|false                   (default false)
    explore:=true|false                (default true  — run CFPA2 single-robot)
    cleanup_stale:=true|false          (default true)
    spawn_x, spawn_y, spawn_yaw        (per-scene defaults below)
    astar_config:=/path/to/yaml        (default = go2w_config/config/nav/astar_nav_go2w.yaml)

Examples:
    ros2 launch go2_gazebo_sim single_astar_mujoco.launch.py
    ros2 launch go2_gazebo_sim single_astar_mujoco.launch.py robot:=go2 scene:=demo3 gui:=true rviz:=true
"""
from __future__ import annotations

import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

from xml.dom import minidom

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack


NS = "robot"  # single-robot namespace used by ros_control_go2{w,}_robot.yaml


def _patch_urdf_for_mujoco(urdf_str: str) -> str:
    """Swap the gazebo_ros2_control plugin for mujoco_ros2_control's,
    strip <gazebo> blocks, and inject a `max` attribute on velocity
    command_interfaces.

    Go2W's wheel joints use `<command_interface name="velocity"/>` with no
    `max`. mujoco_ros2_control's `registerJoints` has the URDF-based
    velocity_limit lookup commented out (mujoco_system.cpp:113-126), so
    velocity_limit falls back to `string_to_double(command_interface.max, 2.0)`.
    Without a max on the URDF it caps at 2 rad/s → ~0.17 m/s wheel cap —
    the robot can't cruise. Inject max="30" (matches <limit velocity="30.1">
    on the foot joints) so the wheel velocity command passes through.
    """
    doc = minidom.parseString(urdf_str)
    for hw in doc.getElementsByTagName("hardware"):
        for plugin in hw.getElementsByTagName("plugin"):
            for child in plugin.childNodes:
                if child.nodeType == child.TEXT_NODE and "GazeboSystem" in child.data:
                    child.data = child.data.replace(
                        "gazebo_ros2_control/GazeboSystem",
                        "mujoco_ros2_control/MujocoSystem",
                    )
    # Add <param name="max">30</param> to every <command_interface
    # name="velocity"/>.  ros2_control humble's parse_interfaces_from_xml
    # reads min/max from CHILD <param> elements (component_parser.cpp:258),
    # not from attributes. Without this, mujoco_system's velocity_limit
    # falls back to `string_to_double(command_interface.max, 2.0)` = 2.0,
    # which clamps wheel commands to 0.17 m/s and stops Go2W from cruising.
    for ci in doc.getElementsByTagName("command_interface"):
        if ci.getAttribute("name") != "velocity":
            continue
        existing = {p.getAttribute("name") for p in ci.getElementsByTagName("param")}
        for pname, pval in (("max", "30.0"), ("min", "-30.0")):
            if pname in existing:
                continue
            p = doc.createElement("param")
            p.setAttribute("name", pname)
            p.appendChild(doc.createTextNode(pval))
            ci.appendChild(p)
    # Strip any <gazebo> blocks that include libgazebo* plugins — they
    # crash mujoco_ros2_control on parse.
    for gz in list(doc.getElementsByTagName("gazebo")):
        has_libgazebo = any(
            "libgazebo" in (p.getAttribute("filename") or "")
            for p in gz.getElementsByTagName("plugin")
        )
        if has_libgazebo:
            gz.parentNode.removeChild(gz)
    return doc.toxml()


def _as_bool(v: str) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes", "on"}


def _get(ctx, key: str) -> str:
    return LaunchConfiguration(key).perform(ctx)


def _build_cleanup_stale_cmd() -> str:
    """Kill leftover sim/nav procs that survive a prior run and would
    pollute DDS discovery or block controller_manager."""
    patterns = [
        "ros2 launch go2_gazebo_sim single_astar",
        "ros2 launch go2_gazebo_sim single_mppi",
        "ros2 launch go2_gazebo_sim nav_test_mujoco",
        "mujoco_ros2_control",
        "/mujoco_sensor_bridge/",
        "/champ_base/",
        "/fast_lio/",
        "/mppi_nav",
        "/octomap_server/",
        "/cfpa2_collaborative_autonomy/",
        "/robot_state_publisher",
        "/robot_localization/",
        "/opt/ros/.*/lib/controller_manager/spawner",
    ]
    cmd = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]
    for p in patterns:
        cmd.append(f"kill_pattern '{p}' TERM; ")
    cmd.append("sleep 1; ")
    for p in patterns:
        cmd.append(f"kill_pattern '{p}' KILL; ")
    cmd.append(
        "rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* "
        "/dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true; "
    )
    cmd.append("sleep 0.5")
    return "".join(cmd)


def _scene_defaults(robot: str, scene: str, pkg_share: str) -> tuple[str, float, float]:
    """Pick (mjcf_path, default_spawn_x, default_spawn_y) for a robot+scene combo."""
    mjcf_map = {
        ("go2w", "demo1"): "demo1.xml",
        ("go2w", "demo3"): "demo3.xml",
        ("go2",  "demo1"): "demo1_go2.xml",
        ("go2",  "demo3"): "demo3_go2.xml",
    }
    if (robot, scene) not in mjcf_map:
        raise ValueError(f"Unsupported robot/scene combo: {robot}/{scene}")
    mjcf = os.path.join(pkg_share, "mujoco", mjcf_map[(robot, scene)])
    # demo1 = 12×8 m, demo3 = 24×16 m; both spawn central-ish.
    spawn = {"demo1": (4.0, 0.0), "demo3": (4.0, 2.0)}[scene]
    return mjcf, spawn[0], spawn[1]


def _build_sensor_bridges(mjcf_path: str, use_sim_time: bool, base_body: str,
                          links_config: str, pose_sensor: str, imu_sensor: str):
    """MuJoCo → /robot/odom/ground_truth + /robot/imu/data + /robot/foot_contacts."""
    return [
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace=NS,
            name="mujoco_odom_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "mjcf_path": mjcf_path,
                "publish_rate": 50.0,
                "base_body_name": base_body,
                "odom_frame": "odom",
                "base_frame": base_body,
                "publish_tf": True,  # seeds Fast-LIO with odom→base identity
                "pose_topic": f"/{NS}/{pose_sensor}/pose",
                "imu_topic": f"/{NS}/{imu_sensor}/imu",
                "republish_imu_topic": "imu/data",
            }],
            remappings=[("/tf", f"/{NS}/tf"), ("/tf_static", f"/{NS}/tf_static")],
            output="screen",
        ),
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace=NS,
            name="mujoco_contact_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mjcf_path},
                {"publish_rate": 50.0},
                links_config,
            ],
            output="screen",
        ),
    ]


def _build_slam_stack(*, use_sim_time: bool, go2w_config_pkg: str,
                      slam_delay: float, base_frame: str):
    """Fast-LIO2 + octomap + scan converter. Returns a list of actions."""
    tf_remaps = [("/tf", f"/{NS}/tf"), ("/tf_static", f"/{NS}/tf_static")]
    slam_config = os.path.join(go2w_config_pkg, "config", "slam", "pointlio_gazebo.yaml")
    actions = []

    # BE LiDAR → Reliable (Fast-LIO needs Reliable)
    actions.append(Node(
        package="go2w_perception",
        executable="qos_bridge.py",
        namespace=NS,
        name="qos_bridge",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": f"/{NS}/mujoco_lidar_sensor/registered_scan",
            "output_topic": f"/{NS}/registered_scan_reliable",
            "input_reliability": "best_effort",
            "output_reliability": "reliable",
        }],
        output="screen",
    ))

    # registered_scan → velodyne_points (Fast-LIO expects velodyne layout)
    actions.append(Node(
        package="go2w_perception",
        executable="pointcloud_adapter.py",
        namespace=NS,
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{NS}/registered_scan_reliable"},
            {"output_topic": f"/{NS}/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
    ))

    # 3D cloud → 2D scan for MPPI's safety layer
    actions.append(Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        namespace=NS,
        name="pc_to_laserscan",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": base_frame,
            "transform_tolerance": 0.1,
            "min_height": 0.05,
            "max_height": 0.60,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.3,
            "range_max": 30.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", f"/{NS}/registered_scan_reliable"),
            ("scan", f"/{NS}/scan_3d"),
        ] + tf_remaps,
        output="screen",
    ))

    # Static TFs: world ≡ map ≡ odom.  Fast-LIO will own odom → base_frame.
    for parent, child in [("world", "map"), ("map", "odom")]:
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=NS,
            name=f"{parent}_to_{child}_tf",
            arguments=[
                "--frame-id", parent, "--child-frame-id", child,
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            remappings=[("/tf_static", f"/{NS}/tf_static")],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ))

    # Fast-LIO2 + slam_odom_relay (T = slam_delay)
    slam_nodes = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            namespace=NS,
            name="slam_node",
            parameters=[slam_config, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/velodyne_points", f"/{NS}/velodyne_points"),
                ("/imu/data", f"/{NS}/imu/data"),
                ("/Odometry", f"/{NS}/Odometry"),
                ("/cloud_registered_body", f"/{NS}/cloud_registered_body"),
                # Sink Fast-LIO's /tf so it doesn't fight for `body`'s parent.
                ("/tf", f"/{NS}/fastlio_tf_sink"),
                ("/tf_static", f"/{NS}/tf_static"),
            ],
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="slam_odom_relay.py",
            namespace=NS,
            name="slam_odom_relay",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": f"/{NS}/Odometry",
                "gt_topic": f"/{NS}/odom/ground_truth",
                "output_topic": f"/{NS}/odom/nav",
                "output_frame_id": "world",
                "output_child_frame_id": base_frame,
                "bootstrap_from_gt": True,
                "require_gt_for_alignment": True,
            }],
            remappings=tf_remaps,
            output="screen",
        ),
    ]
    actions.append(TimerAction(period=slam_delay, actions=slam_nodes))

    # Octomap → /robot/map (OccupancyGrid)
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        namespace=NS,
        name="octomap_server",
        parameters=[{
            "use_sim_time": use_sim_time,
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": base_frame,
            "sensor_model.max_range": 6.0,
            # Aggressive hit probability (was 0.8) + conservative miss (was 0.35)
            # to prevent A* from finding phantom gaps in thin walls (e.g.
            # demo1 divider_v_north is 0.15 m thick and can look sparse
            # under a single LiDAR sweep). One hit ≈ certain occupancy,
            # and clear-evidence-only markdown keeps unknown → unknown.
            "sensor_model.hit": 0.95,
            "sensor_model.miss": 0.12,
            "sensor_model.min": 0.12,
            "sensor_model.max": 0.97,
            "point_cloud_min_z": 0.20,
            "point_cloud_max_z": 1.10,
            "occupancy_min_z": 0.20,
            "occupancy_max_z": 1.00,
            "filter_ground_plane": False,
            "filter_speckles": False,
            "compress_map": True,
            "latch": True,
            "publish_free_space": False,
        }],
        remappings=[
            ("cloud_in", f"/{NS}/registered_scan_reliable"),
            ("projected_map", f"/{NS}/map"),
        ] + tf_remaps,
        output="screen",
    )
    actions.append(TimerAction(period=slam_delay + 1.0, actions=[octomap_node]))

    return actions


def _build_astar_stack(*, use_sim_time: bool, astar_config_path: str,
                      nav_delay: float, has_wheels: bool, go2w_config_pkg: str,
                      nav_backend: str = "astar"):
    """Nav planner + twist_bridge + (Go2W only) hybrid_cmd_router.

    nav_backend selects the planner executable:
      astar              → astar_nav_node
      hybrid_astar       → hybrid_astar_nav_node          (our v0.1)
      nav2_hybrid_astar  → nav2_hybrid_astar_nav_node     (B-route)
    """
    _exe = {
        "astar":             ("astar_nav_node",            "astar_nav"),
        "hybrid_astar":      ("hybrid_astar_nav_node",     "hybrid_astar_nav"),
        "nav2_hybrid_astar": ("nav2_hybrid_astar_nav_node", "nav2_hybrid_astar_nav"),
    }
    nav_executable, nav_node_name = _exe[nav_backend]
    tf_remaps = [("/tf", f"/{NS}/tf"), ("/tf_static", f"/{NS}/tf_static")]

    nav_remaps = [
        ("/way_point", f"/{NS}/way_point_coord"),   # CFPA2 publishes here
        ("/odom/ground_truth", f"/{NS}/odom/nav"),  # MPPI's odom
        ("/scan", f"/{NS}/scan_3d"),
        ("/cmd_vel_stamped", f"/{NS}/cmd_vel_stamped"),
        ("/nav_status", f"/{NS}/nav_status"),
        ("/planned_path", f"/{NS}/planned_path"),
        ("/global_planned_path", f"/{NS}/global_planned_path"),
        ("/robot_trajectory", f"/{NS}/robot_trajectory"),
        ("/final_goal_marker", f"/{NS}/final_goal_marker"),
        ("/robot_pose_marker", f"/{NS}/robot_pose_marker"),
        ("/frontier_replan", f"/{NS}/frontier_replan"),
        ("/stop", f"/{NS}/stop"),
    ] + tf_remaps

    astar_node = Node(
        package="go2w_nav",
        executable=nav_executable,
        namespace=NS,
        name=nav_node_name,
        parameters=[
            astar_config_path,
            {"use_sim_time": use_sim_time},
            {
                "map_frame": "map",
                "map_topic": f"/{NS}/map",
                "frontier_replan_topic": f"/{NS}/frontier_replan",
                "stop_topic": f"/{NS}/stop",
            },
        ],
        remappings=nav_remaps,
        output="screen",
    )

    # cmd_vel_stamped → cmd_vel (for CHAMP and hybrid_router)
    twist_bridge = Node(
        package="go2w_perception",
        executable="twist_bridge.py",
        namespace=NS,
        name="twist_bridge",
        remappings=[
            ("/cmd_vel_stamped", f"/{NS}/cmd_vel_stamped"),
            ("/cmd_vel", f"/{NS}/cmd_vel"),
        ],
        output="screen",
    )

    actions = [TimerAction(period=nav_delay, actions=[astar_node, twist_bridge])]

    if has_wheels:
        # Go2W: cmd_vel → hybrid_router → (cmd_vel_legged + wheel_velocity_controller)
        # Note: the router's default wheel_command_topic is
        # `wheel_velocity_controller/commands`, but our ros_control_go2w_robot.yaml
        # names the controller `robot_wheel_velocity_controller`, so the actual
        # topic is `/{ns}/robot_wheel_velocity_controller/commands`. Override the
        # param here so the router publishes to the correct topic.
        hybrid_router = Node(
            package="go2w_control",
            executable="go2w_hybrid_cmd_router.py",
            namespace=NS,
            name="go2w_hybrid_cmd_router",
            parameters=[
                os.path.join(
                    go2w_config_pkg, "config", "control", "go2w_hybrid_motion.yaml",
                ),
                {
                    "use_sim_time": use_sim_time,
                    "wheel_command_topic": "robot_wheel_velocity_controller/commands",
                },
            ],
            output="screen",
        )
        actions.append(TimerAction(period=nav_delay, actions=[hybrid_router]))
    # else: Go2 — CHAMP subscribes directly to /robot/cmd_vel (wired below).

    return actions


def _launch_setup(context):
    use_sim_time = True
    robot = _get(context, "robot").strip().lower()
    scene = _get(context, "scene").strip().lower()
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    spawn_x_arg = _get(context, "spawn_x").strip()
    spawn_y_arg = _get(context, "spawn_y").strip()
    spawn_yaw = _get(context, "spawn_yaw").strip() or "0.0"
    astar_config_arg = _get(context, "astar_config").strip()
    nav_backend = _get(context, "nav_backend").strip().lower() or "astar"
    if nav_backend == "hybrid":
        nav_backend = "hybrid_astar"
    if nav_backend == "nav2":
        nav_backend = "nav2_hybrid_astar"
    if nav_backend not in {"astar", "hybrid_astar", "nav2_hybrid_astar"}:
        raise ValueError(
            f"nav_backend must be 'astar' | 'hybrid_astar' | 'nav2_hybrid_astar'; "
            f"got '{nav_backend}'")

    # Bench plumbing: session_reporter counts wall contacts, computes
    # coverage, writes JSON. Wall checker is a terminal fail-early
    # (shuts launch on first hit) — off by default because during
    # benchmarking we want the whole-run contact count.
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_path = _get(context, "session_output_path").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    enable_wall_checker = _as_bool(_get(context, "enable_wall_checker"))

    if robot not in ("go2w", "go2"):
        raise ValueError(f"robot must be 'go2w' or 'go2', got {robot!r}")
    if scene not in ("demo1", "demo3"):
        raise ValueError(f"scene must be 'demo1' or 'demo3', got {scene!r}")

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    champ_base_pkg = get_package_share_directory("champ_base")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    mjcf_path, default_x, default_y = _scene_defaults(robot, scene, go2_gazebo_pkg)
    spawn_x = spawn_x_arg or str(default_x)
    spawn_y = spawn_y_arg or str(default_y)

    # Robot-specific configs
    if robot == "go2w":
        urdf_xacro = os.path.join(
            go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"
        )
        ros_control_yaml = os.path.join(
            go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot.yaml"
        )
        has_wheels = True
        cmd_vel_input = "cmd_vel_legged"      # goes via hybrid_cmd_router
        wheel_controller = "robot_wheel_velocity_controller"
    else:  # go2
        urdf_xacro = os.path.join(
            go2_gazebo_pkg, "urdf", "go2", "go2_description_3d_lidar.xacro"
        )
        ros_control_yaml = os.path.join(
            go2_gazebo_pkg, "config", "ros_control", "ros_control_go2_robot.yaml"
        )
        has_wheels = False
        cmd_vel_input = "cmd_vel"             # direct to CHAMP, no router
        wheel_controller = None

    robot_description = xacro.process_file(urdf_xacro).documentElement.toxml()
    robot_description = _patch_urdf_for_mujoco(robot_description)

    # CHAMP configs — Go2W and Go2 share the same gait/links/joints tuning file
    # (Go2's ros_control yaml omits the wheel block but CHAMP itself is agnostic).
    joints_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    links_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    # Nav config + executable per backend.
    _backend_yaml = {
        "astar":             "astar_nav_go2w.yaml",
        "hybrid_astar":      "hybrid_astar_nav_go2w.yaml",
        "nav2_hybrid_astar": "nav2_hybrid_astar_nav_go2w.yaml",
    }[nav_backend]
    astar_config_path = astar_config_arg or os.path.join(
        go2w_config_pkg, "config", "nav", _backend_yaml
    )

    mujoco_plugin_dir = _find_mujoco_plugin_dir()

    actions = [LogInfo(msg=(
        f"[single_mppi_mujoco] robot={robot}  scene={scene}  "
        f"spawn=({spawn_x},{spawn_y},{spawn_yaw})  mjcf={os.path.basename(mjcf_path)}"
    ))]

    # T=0: cleanup stale procs
    if cleanup_stale:
        actions.append(ExecuteProcess(
            cmd=["bash", "-lc", _build_cleanup_stale_cmd()], output="screen",
        ))

    # T=3: MuJoCo plugin under the ROBOT namespace so that its
    # controller_manager becomes /robot/controller_manager — matching what
    # ros_control_go2{w,}_robot.yaml's keys expect and what the spawners
    # look for. Sensor topics also land under /robot/... (not /mujoco_sim).
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=NS,
        parameters=[
            {"robot_description": robot_description},
            ros_control_yaml,
            {"robot_model_path": mjcf_path},
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
        ],
        remappings=[
            (f"/{NS}/controller_manager/robot_description", f"/{NS}/robot_description"),
        ],
        additional_env={"MUJOCO_PLUGIN_DIR": mujoco_plugin_dir},
        output="screen",
    )
    actions.append(TimerAction(period=3.0, actions=[mujoco_node]))

    # T=5: sensor bridges (odom + foot contacts)
    # The Go2W/Go2 demo MJCFs use base_link as the base body, imu as the IMU
    # site, and `base_link_site_pose_sensor`/`imu_imu_sensor` as published names.
    base_body = "base_link"
    actions.append(TimerAction(
        period=5.0,
        actions=_build_sensor_bridges(
            mjcf_path=mjcf_path,
            use_sim_time=use_sim_time,
            base_body=base_body,
            links_config=links_config,
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
        ),
    ))

    # T=7: CHAMP locomotion stack (RSP + quadruped_controller + spawners + stand_up)
    robot_stack = build_dual_robot_stack(
        ns=NS,
        spawn_x=spawn_x, spawn_y=spawn_y, spawn_yaw=spawn_yaw,
        use_sim_time=use_sim_time,
        robot_description=robot_description,
        joints_config=joints_config,
        links_config=links_config,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        cmd_vel_input_topic=cmd_vel_input,
        wheel_controller_name=wheel_controller,
        use_mujoco=True,
        # Single-robot CM: the ros_control yaml keys everything under /robot.
        controller_manager_name=f"/{NS}/controller_manager",
    )
    actions.append(TimerAction(period=7.0, actions=robot_stack))

    # T=20: Fast-LIO + octomap + scan conversion
    slam_delay = 20.0
    nav_delay = slam_delay + 5.0  # MPPI starts after octomap has one map out
    actions.extend(_build_slam_stack(
        use_sim_time=use_sim_time,
        go2w_config_pkg=go2w_config_pkg,
        slam_delay=slam_delay,
        base_frame=base_body,
    ))

    # T=25: MPPI + twist_bridge (+ hybrid_router if Go2W)
    actions.extend(_build_astar_stack(
        use_sim_time=use_sim_time,
        astar_config_path=astar_config_path,
        nav_delay=nav_delay,
        has_wheels=has_wheels,
        go2w_config_pkg=go2w_config_pkg,
        nav_backend=nav_backend,
    ))

    # T=nav_delay+2: CFPA2 single-robot (publishes /robot/way_point_coord)
    if explore:
        cfpa2_config = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
        actions.append(TimerAction(
            period=nav_delay + 2.0,
            actions=[Node(
                package="cfpa2_collaborative_autonomy",
                executable="cfpa2_coordinator_node",
                name="cfpa2_coordinator",
                parameters=[
                    cfpa2_config,
                    {
                        "use_sim_time": use_sim_time,
                        "namespaces": [NS],
                        "goal_topic_suffix": "/way_point_coord",
                        "marker_frame_override": "map",
                    },
                ],
                output="screen",
            )],
        ))

    # ── Wall / tip-over fail checker (optional — terminal) ──
    # Subscribes to /mujoco/contacts and /{ns}/odom/ground_truth; exits non-
    # zero on first robot-vs-wall contact or tip-over (|roll|, |pitch| > 45°).
    # OnProcessExit then shuts the launch down so the failure is terminal.
    # Off by default during benchmarks — we want the full-run contact count.
    if enable_wall_checker:
        wall_checker_script = os.path.expanduser(
            "~/Research/Collab_QRC/scripts/runtime/far_wall_checker.py"
        )
        wall_checker_proc = ExecuteProcess(
            cmd=["python3", "-u", wall_checker_script],
            name="far_wall_checker",
            output="screen",
        )
        actions.append(TimerAction(
            period=nav_delay + 3.0,
            actions=[wall_checker_proc],
        ))
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=wall_checker_proc,
                on_exit=[
                    LogInfo(msg="far_wall_checker exited — shutting down "
                                "launch (robot hit a wall or tipped)"),
                    Shutdown(reason="far_wall_checker detected failure"),
                ],
            )
        ))

    # ── Bounded session reporter (JSON + contact counter) ──
    # Same script the FAR benchmark uses. Counts wall contacts from
    # /mujoco/contacts, tracks tip-over, explored area via /{ns}/map,
    # dumps JSON on exit. When it finishes, shut the launch down.
    if session_duration_sec > 0.0:
        if not session_output_path:
            session_output_path = "/tmp/session_reports/astar_latest.json"
        session_script = os.path.expanduser(
            "~/Research/Collab_QRC/scripts/bench/session_reporter.py"
        )
        session_proc = ExecuteProcess(
            cmd=[
                "python3", "-u", session_script,
                "--duration", str(session_duration_sec),
                "--namespace", NS,
                "--output", session_output_path,
                "--scene-area-m2", str(scene_area_m2),
            ],
            name="session_reporter",
            output="screen",
        )
        actions.append(TimerAction(
            period=nav_delay + 3.0,
            actions=[session_proc],
        ))
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=session_proc,
                on_exit=[
                    LogInfo(msg="session_reporter exited — shutting down "
                                "launch (bounded session complete)"),
                    Shutdown(reason="session_reporter session complete"),
                ],
            )
        ))

    # RViz — namespace TF visibility + strip snap-injected env vars
    if rviz:
        actions.append(TimerAction(
            period=slam_delay,
            actions=[
                Node(
                    package="go2w_perception",
                    executable="multi_tf_relay",
                    name="multi_tf_relay",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"sources": [NS]},
                    ],
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "bash", "-c",
                        "unset XDG_DATA_HOME GSETTINGS_SCHEMA_DIR GTK_PATH LOCPATH "
                        "SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_REVISION "
                        "SNAP_LIBRARY_PATH SNAP_USER_DATA SNAP_USER_COMMON; "
                        "exec rviz2 -d \"$1\" "
                        "--ros-args -p use_sim_time:=true "
                        "--log-level rviz2:=WARN "
                        "--log-level rviz_common:=WARN "
                        "--log-level rviz_default_plugins:=WARN",
                        "--",
                        # nav_test.rviz is the stock single-robot rviz config
                        # using /robot/* topics. If it doesn't exist the user
                        # can point to any valid .rviz.
                        os.path.join(go2_gazebo_pkg, "rviz", "nav_test.rviz"),
                    ],
                    name="rviz2_single_mppi",
                    output="log",
                ),
            ],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2w",
                              description="go2w | go2"),
        DeclareLaunchArgument("scene", default_value="demo3",
                              description="demo1 (12×8 m) | demo3 (24×16 m)"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value=""),
        DeclareLaunchArgument("spawn_y", default_value=""),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "astar_config", default_value="",
            description="Path to A* nav params yaml; default depends on nav_backend",
        ),
        DeclareLaunchArgument(
            "nav_backend", default_value="astar",
            description="Nav planner: 'astar' (default) | 'hybrid_astar' (Hybrid A* + Ceres)"
                        " | 'nav2_hybrid_astar' (B-route: nav2_smac_planner lib)."
                        " Aliases: hybrid → hybrid_astar, nav2 → nav2_hybrid_astar.",
        ),
        DeclareLaunchArgument(
            "session_duration_sec", default_value="0",
            description="If > 0, run session_reporter for N seconds and "
                        "shut launch down after (for bench/smoke runs).",
        ),
        DeclareLaunchArgument(
            "session_output_path",
            default_value="/tmp/session_reports/astar_latest.json",
            description="JSON output path for session_reporter.",
        ),
        DeclareLaunchArgument(
            "scene_area_m2", default_value="96.0",
            description="Ground-truth observable scene area in m² (for "
                        "coverage ratio). demo1=96, demo3≈384.",
        ),
        DeclareLaunchArgument(
            "enable_wall_checker", default_value="false",
            description="Terminal fail-early wall/tip-over checker. Off "
                        "for bench runs; set true during development.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
