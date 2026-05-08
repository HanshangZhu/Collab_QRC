#!/usr/bin/env python3
"""Dual-robot MuJoCo + Fast-LIO2 + FAR nav benchmark launch.

Two Go2W robots share one MuJoCo process and one combined URDF. Each
robot gets its own Fast-LIO2 SLAM, octomap, FAR planner, and nav stack
in its own namespace (`robot_a` / `robot_b`). A shared CFPA2
coordinator partitions frontier goals across both robots. A shared
inter-robot collision monitor subscribes to `/mujoco/contacts` and
logs any A↔B geom-pair contact to a JSON report.

Assumes the MJCF has both robots pre-defined: Robot A with bare joint
names (FL_hip_joint, base_link, ...), Robot B with `b_` prefix on
every joint / site / body. The default scene is
`demo3_dual.xml` (24×16m with two Go2Ws at (4, 2) and (4, -6)).
"""
from __future__ import annotations

import os
import sys

# sys.path must be amended BEFORE the `from modules.*` imports below — when
# ros2 launch loads this file it doesn't add the launch dir to sys.path.

_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

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
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.dual_urdf import build_robot_b_urdf
from modules.dual_urdf_nav import build_dual_nav_urdf


def _as_bool(v: str) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes", "on"}


def _get(ctx, key: str) -> str:
    return LaunchConfiguration(key).perform(ctx)


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    return data


def _build_cleanup_stale_cmd() -> str:
    """Kill any leftover sim/nav processes from a prior run.

    Pattern list mirrors benchmark_fastlio.sh's cleanup_procs so we cover
    everything that could pollute DDS discovery and break controller_manager
    service lookup in the next launch.
    """
    # Target only processes known to leak between runs. Previously included
    # "/go2w_perception/" but that patterned-killed this same launch's qos_bridge
    # and pointcloud_adapter when they spawned at T=0 alongside cleanup_stale —
    # breaking the LiDAR → Fast-LIO → octomap → /map → CFPA2 pipeline from
    # step 1. External benchmark scripts (benchmark_fastlio.sh) handle stale
    # perception procs before launching; no need to duplicate here.
    patterns = [
        "ros2 launch go2_gazebo_sim nav_test_mujoco",
        "mujoco_ros2_control",
        "/mujoco_sensor_bridge/",
        "/champ_base/",
        "/fast_lio/",
        "/far_planner/",
        "/local_planner/",
        "/terrain_analysis",
        "/octomap_server/",
        "/cfpa2_collaborative_autonomy/",
        "/robot_state_publisher",
        "/robot_localization/",
        "/opt/ros/.*/lib/controller_manager/spawner",
        "session_reporter.py",
        "dual_robot_collision_monitor.py",
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


def _build_sensor_bridges(ns: str, mjcf_path: str, base_body: str, imu_site: str,
                          pose_sensor: str, imu_sensor: str, links_config: str,
                          use_sim_time: bool):
    """Per-robot MuJoCo sensor bridges (ground-truth odom + foot contacts).

    The mujoco plugin publishes raw sensor topics under its own namespace
    (`/mujoco_sim/...`). We subscribe to those in the per-robot bridge
    and republish under /{ns}/odom/ground_truth, /{ns}/imu/data,
    /{ns}/foot_contacts.
    """
    return [
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace=ns,
            name="mujoco_odom_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "mjcf_path": mjcf_path,
                "publish_rate": 50.0,
                "base_body_name": base_body,
                "odom_frame": "odom",
                "base_frame": base_body,
                # Fast-LIO needs to own the map→odom→base_link chain.
                # Setting publish_tf=True here gives an early odom→base_link
                # identity TF so Fast-LIO has something to seed from.
                "publish_tf": True,
                "pose_topic": f"/mujoco_sim/{pose_sensor}/pose",
                "imu_topic": f"/mujoco_sim/{imu_sensor}/imu",
                "republish_imu_topic": "imu/data",
            }],
            remappings=[
                ("/tf", f"/{ns}/tf"),
                ("/tf_static", f"/{ns}/tf_static"),
            ],
            output="screen",
        ),
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace=ns,
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


def _build_fastlio_nav_stack(
    *,
    ns: str,
    mujoco_lidar_topic: str,
    base_frame: str,
    imu_frame: str,
    use_sim_time: bool,
    nav_backend: str,
    slam_delay: float,
    nav_delay: float,
    go2w_config_pkg: str,
    local_planner_paths_dir: str,
    far_tuning_yaml: str,
    far_default_yaml: str,
):
    """Per-robot Fast-LIO + octomap + FAR nav stack.

    Returns a list of Node / TimerAction actions. Keep them grouped
    under TimerActions so they start in the right order:
      T=slam_delay        — Fast-LIO, octomap, static TFs
      T=slam_delay+0.5    — pointcloud_frame_bridge (body → map)
      T=nav_delay         — FAR stack (terrain_analysis × 2, far_planner,
                            localPlanner, pathFollower)
    """
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    actions = []

    # ── QoS bridge: BE LiDAR → Reliable (Fast-LIO needs Reliable) ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="qos_bridge.py",
            namespace=ns,
            name="qos_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": mujoco_lidar_topic,
                "output_topic": f"/{ns}/registered_scan_reliable",
                "input_reliability": "best_effort",
                "output_reliability": "reliable",
            }],
            output="screen",
        )
    )

    # ── pointcloud_adapter: registered_scan → velodyne_points for Fast-LIO ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="pointcloud_adapter.py",
            namespace=ns,
            name="pointcloud_adapter",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": f"/{ns}/registered_scan_reliable"},
                {"output_topic": f"/{ns}/velodyne_points"},
                {"num_rings": 16},
            ],
            output="screen",
        )
    )

    # ── pointcloud_to_laserscan: 3D → 2D for visualisation / secondary use ──
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=ns,
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
                ("cloud_in", f"/{ns}/registered_scan_reliable"),
                ("scan", f"/{ns}/scan_3d"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── Static TFs: world ≡ map ≡ odom ≡ body (all identity to base) ──
    # Fast-LIO publishes the /cloud_registered_body PointCloud2 with
    # header.frame_id="body" regardless of URDF. pointcloud_frame_bridge
    # needs to resolve `body → map` — if missing, registered_scan_map →
    # terrain_analysis → FAR never produces output and robots can't move.
    # The per-robot URDF's `imu` link is NOT connected to `base_link` via
    # robot_state_publisher (the xacro has `imu` as an orphan), so we can
    # NOT chain through imu. Attach `body` directly to the per-robot
    # base frame (base_link or b_base_link).
    for parent, child in [("world", "map"), ("map", "odom"), (base_frame, "body")]:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=ns,
                name=f"{parent}_to_{child}_tf".replace("-", "_"),
                arguments=[
                    "--frame-id", parent, "--child-frame-id", child,
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                remappings=[("/tf_static", f"/{ns}/tf_static")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    # ── Fast-LIO2 SLAM ──
    slam_config = os.path.join(go2w_config_pkg, "config", "slam", "pointlio_gazebo.yaml")
    slam_nodes = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            namespace=ns,
            name="slam_node",
            parameters=[slam_config, {"use_sim_time": use_sim_time}],
            # Fast-LIO hard-codes a `camera_init -> body` TF
            # (laserMapping.cpp:654). Letting it hit /{ns}/tf gives `body`
            # two parents (ours: base_link, Fast-LIO's: camera_init) and
            # breaks `body -> map` lookup. Route Fast-LIO's /tf to a sink;
            # nobody consumes it. /tf_static is still shared normally.
            remappings=[
                ("/velodyne_points", f"/{ns}/velodyne_points"),
                ("/imu/data", f"/{ns}/imu/data"),
                ("/Odometry", f"/{ns}/Odometry"),
                ("/cloud_registered_body", f"/{ns}/cloud_registered_body"),
                ("/tf", f"/{ns}/fastlio_tf_sink"),
                ("/tf_static", f"/{ns}/tf_static"),
            ],
            output="screen",
        ),
        # slam_odom_relay: renames Fast-LIO's Odometry topic for nav consumption
        Node(
            package="go2w_perception",
            executable="slam_odom_relay.py",
            namespace=ns,
            name="slam_odom_relay",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": f"/{ns}/Odometry",
                "gt_topic": f"/{ns}/odom/ground_truth",
                "output_topic": f"/{ns}/odom/nav",
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

    # ── pointcloud_frame_bridge: body-frame Fast-LIO cloud → map frame for FAR ──
    actions.append(
        TimerAction(
            period=slam_delay + 0.5,
            actions=[
                Node(
                    package="go2w_perception",
                    executable="pointcloud_frame_bridge.py",
                    namespace=ns,
                    name="registered_scan_frame_bridge",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": f"/{ns}/cloud_registered_body"},
                        {"output_topic": f"/{ns}/registered_scan_map"},
                        {"target_frame": "map"},
                        {"tf_timeout_sec": 0.15},
                        {"transform_wait_sec": 0.10},
                        {"max_cloud_age_sec": 0.80},
                    ],
                    remappings=tf_remaps,
                    output="screen",
                ),
            ],
        )
    )

    # ── Octomap: /{ns}/map OccupancyGrid from Fast-LIO cloud ──
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        namespace=ns,
        name="octomap_server",
        parameters=[{
            "use_sim_time": use_sim_time,
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": base_frame,
            "sensor_model.max_range": 6.0,
            "sensor_model.hit": 0.8,
            "sensor_model.miss": 0.35,
            "sensor_model.min": 0.12,
            "sensor_model.max": 0.97,
            "point_cloud_min_z": 0.20,
            "point_cloud_max_z": 1.10,
            "occupancy_min_z": 0.20,
            "occupancy_max_z": 1.00,
            # filter_ground_plane would need min_z <= 0 to see ground; our
            # min_z=0.20 already excludes the floor, so leave ground-filter
            # off (it spams "No ground plane found in scan" at 10 Hz otherwise).
            "filter_ground_plane": False,
            "filter_speckles": False,
            "compress_map": True,
            "latch": True,
            "publish_free_space": False,
        }],
        remappings=[
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("projected_map", f"/{ns}/map"),
        ] + tf_remaps,
        output="screen",
    )
    actions.append(TimerAction(period=slam_delay + 1.0, actions=[octomap_node]))

    if nav_backend != "far":
        return actions  # rrt_star path could be added later; not needed for now

    # ── FAR stack: sensor_scan_generation, terrain_analysis ×2, far_planner,
    #    localPlanner, pathFollower ──
    far_scan_topic = f"/{ns}/registered_scan_map"
    far_odom_topic = f"/{ns}/odom/nav"
    far_max_speed = 0.2

    far_nodes = [
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            namespace=ns,
            name="sensor_scan_generation",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ] + tf_remaps,
            output="screen",
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace=ns,
            name="terrain_analysis",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            namespace=ns,
            name="terrain_analysis_ext",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        ),
        Node(
            package="far_planner",
            executable="far_planner",
            namespace=ns,
            name="far_planner",
            parameters=[
                _load_yaml_params(far_default_yaml),
                far_tuning_yaml,
                {
                    "use_sim_time": use_sim_time,
                    # robot_id is an unused launch param in FAR source
                    # (see graph_msger.cpp:99) — set for future-proofing.
                    "graph_msger/robot_id": 0 if ns == "robot_a" else 1,
                },
            ],
            remappings=[
                ("/odom_world", far_odom_topic),
                ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
                ("/scan_cloud", f"/{ns}/terrain_map"),
                ("/terrain_local_cloud", far_scan_topic),
                ("/goal_point", f"/{ns}/way_point_coord"),
                ("/way_point", f"/{ns}/way_point"),
                ("/joy", f"/{ns}/joy"),
                ("/navigation_boundary", f"/{ns}/navigation_boundary"),
                ("/runtime", f"/{ns}/far_runtime"),
                ("/planning_time", f"/{ns}/far_planning_time"),
                ("/robot_vgraph", f"/{ns}/robot_vgraph"),
                ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
            ] + tf_remaps,
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="localPlanner",
            namespace=ns,
            name="localPlanner",
            parameters=[{
                "use_sim_time": use_sim_time,
                "pathFolder": local_planner_paths_dir,
                "vehicleLength": 0.70,
                "vehicleWidth": 0.40,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": True,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                "checkRotObstacle": True,
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
                ("/way_point", f"/{ns}/way_point"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/overall_map", f"/{ns}/terrain_map"),
                ("/joy", f"/{ns}/joy"),
                ("/path", f"/{ns}/local_path"),
                ("/freePaths", f"/{ns}/free_paths"),
            ],
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="pathFollower",
            namespace=ns,
            name="pathFollower",
            parameters=[{
                "use_sim_time": use_sim_time,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "pubSkipNum": 1,
                "twoWayDrive": True,
                "lookAheadDis": 0.8,
                "yawRateGain": 1.0,
                "stopYawRateGain": 0.8,
                "maxYawRate": 45.0,
                "maxSpeed": far_max_speed,
                "maxAccel": 2.0,
                "switchTimeThre": 1.0,
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
                ("/path", f"/{ns}/local_path"),
                ("/cmd_vel", f"/{ns}/cmd_vel_stamped"),
                ("/joy", f"/{ns}/joy"),
                ("/speed", f"/{ns}/speed"),
                ("/stop", f"/{ns}/stop"),
            ],
            output="screen",
        ),
        # CMU convention: static sensor↔vehicle + sensor↔camera TFs
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_vehicle_tf",
            arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_camera_tf",
            arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
            remappings=tf_remaps,
            output="screen",
        ),
        # twist_bridge: cmd_vel_stamped → cmd_vel for CHAMP
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=ns,
            name="twist_bridge",
            remappings=[
                ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                ("/cmd_vel", f"/{ns}/cmd_vel"),
            ],
            output="screen",
        ),
        # go2w_hybrid_cmd_router: route cmd_vel → legged controller
        Node(
            package="go2w_control",
            executable="go2w_hybrid_cmd_router.py",
            namespace=ns,
            name="go2w_hybrid_cmd_router",
            parameters=[
                os.path.join(
                    go2w_config_pkg, "config", "control",
                    "go2w_hybrid_motion.yaml",
                ),
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
    ]
    actions.append(TimerAction(period=nav_delay, actions=far_nodes))

    return actions


def _launch_setup(context):
    use_sim_time = True
    gui = _as_bool(_get(context, "gui"))
    explore = _as_bool(_get(context, "explore"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_dir = _get(context, "session_output_dir").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    collision_output = _get(context, "collision_output_path").strip()
    nav_backend = "far"  # locked to FAR in dual launch

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    champ_base_pkg = get_package_share_directory("champ_base")
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo3_dual.xml")

    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "config", "ros_control", "ros_control_dual_mujoco_nav.yaml"
    )

    # ── URDF generation ──
    base_robot_description = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    combined_urdf = build_dual_nav_urdf(base_robot_description)
    robot_a_urdf = build_namespaced_robot_description(
        base_robot_description, "robot_a",
        os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_a.yaml"),
    )
    robot_b_urdf = build_robot_b_urdf(base_robot_description)

    # CHAMP configs
    joints_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    joints_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints_robot_b.yaml")
    links_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    links_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links_robot_b.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    far_tuning_yaml = os.path.join(go2w_config_pkg, "config", "nav", "far_planner_tuning.yaml")
    far_default_yaml = os.path.join(far_pkg, "config", "default.yaml")
    local_planner_paths_dir = os.path.join(local_planner_pkg, "paths")

    mujoco_plugin_dir = _find_mujoco_plugin_dir()
    sim_ns = "mujoco_sim"

    actions = [LogInfo(msg="[nav_test_mujoco_fastlio_dual] starting dual-robot nav")]

    # ── T=0: cleanup stale ──
    if cleanup_stale:
        actions.append(
            ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_cmd()], output="screen")
        )

    # ── T=3: MuJoCo (one process with dual URDF) ──
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=sim_ns,
        parameters=[
            {"robot_description": combined_urdf},
            ros2_control_config,
            {"robot_model_path": mujoco_model_path},
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
        ],
        remappings=[
            (f"/{sim_ns}/controller_manager/robot_description", f"/{sim_ns}/robot_description"),
        ],
        additional_env={"MUJOCO_PLUGIN_DIR": mujoco_plugin_dir},
        output="screen",
    )
    actions.append(TimerAction(period=3.0, actions=[mujoco_node]))

    # ── T=5: Sensor bridges (odom + contact) for both robots ──
    sensor_actions = []
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_a", mjcf_path=mujoco_model_path,
            base_body="base_link", imu_site="imu",
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
            links_config=links_a,
            use_sim_time=use_sim_time,
        )
    )
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_b", mjcf_path=mujoco_model_path,
            base_body="b_base_link", imu_site="b_imu",
            pose_sensor="b_base_link_site_pose_sensor",
            imu_sensor="b_imu_imu_sensor",
            links_config=links_b,
            use_sim_time=use_sim_time,
        )
    )
    actions.append(TimerAction(period=5.0, actions=sensor_actions))

    # ── T=7 / T=10: Per-robot CHAMP stacks (staggered to share controller_manager) ──
    # Same pattern as dual_go2w_mujoco_door.launch.py — ROS service calls to
    # controller_manager are serial, so stagger spawners to avoid collisions.
    # Door-launch pattern: wrap each robot's stack in an outer TimerAction
    # so all its sub-actions (RSP, CHAMP, spawners, standup) use their
    # DEFAULT internal sub-delays but start 7s / 10s after launch. This
    # gives mujoco_ros2_control time to come up before controller_manager
    # service calls begin, and staggers A's spawners before B's to avoid
    # load_controller service-timeout races.
    robot_a_stack = build_dual_robot_stack(
        ns="robot_a",
        spawn_x="4.0", spawn_y="2.0", spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_a_urdf,
        joints_config=joints_a, links_config=links_a,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="robot_a_wheel_velocity_controller",
        use_mujoco=True,
        controller_manager_name=f"/{sim_ns}/controller_manager",
    )
    actions.append(TimerAction(period=7.0, actions=robot_a_stack))

    robot_b_stack = build_dual_robot_stack(
        ns="robot_b",
        spawn_x="4.0", spawn_y="-6.0", spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_b_urdf,
        joints_config=joints_b, links_config=links_b,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        # See fastlio_mixed.launch — robot_b joints are b_-prefixed, so
        # stand_up_slowly must prepend "b_" or its trajectory is rejected.
        stand_up_joint_prefix="b_",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="robot_b_wheel_velocity_controller",
        use_mujoco=True,
        controller_manager_name=f"/{sim_ns}/controller_manager",
    )
    actions.append(TimerAction(period=10.0, actions=robot_b_stack))

    # ── Per-robot Fast-LIO + FAR nav stacks ──
    slam_delay = 20.0   # after both standups complete
    nav_delay = slam_delay + 5.0

    actions.extend(
        _build_fastlio_nav_stack(
            ns="robot_a",
            # Plugin publishes under /mujoco_sim/ — the sim's controller_manager
            # namespace — NOT per-robot. Robot A's LiDAR site is "livox_mid360"
            # (no prefix), so the topic is /mujoco_sim/mujoco_lidar_sensor/
            # registered_scan.
            mujoco_lidar_topic="/mujoco_sim/mujoco_lidar_sensor/registered_scan",
            # Robot A's URDF uses bare link names (no prefix) — its TF tree
            # has `base_link` as the root and `imu` as the IMU link.
            base_frame="base_link",
            imu_frame="imu",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend,
            slam_delay=slam_delay,
            nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
        )
    )
    actions.extend(
        _build_fastlio_nav_stack(
            ns="robot_b",
            # Robot B's LiDAR site is "b_livox_mid360" (b_ prefix in MJCF) →
            # plugin names the topic accordingly.
            mujoco_lidar_topic="/mujoco_sim/b_mujoco_lidar_sensor/registered_scan",
            # Robot B's URDF is `b_`-prefixed (via build_robot_b_urdf) →
            # robot_state_publisher emits `b_base_link` into /robot_b/tf.
            # Every downstream consumer (octomap, laser_scan target_frame,
            # slam_odom_relay output child frame) must use that same name
            # or TF lookup fails. IMU link is `b_imu` for the same reason.
            base_frame="b_base_link",
            imu_frame="b_imu",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend,
            slam_delay=slam_delay,
            nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
        )
    )

    # ── CFPA2 dual-robot coordinator (shared) ──
    if explore:
        cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_coordinator.yaml")
        if not os.path.exists(cfpa2_config_path):
            cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
        actions.append(
            TimerAction(
                period=nav_delay + 2.0,
                actions=[
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_coordinator_node",
                        name="cfpa2_coordinator",
                        parameters=[
                            cfpa2_config_path,
                            {
                                "use_sim_time": use_sim_time,
                                "namespaces": ["robot_a", "robot_b"],
                                "goal_topic_suffix": "/way_point_coord",
                                "marker_frame_override": "map",
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── Inter-robot collision monitor (shared) ──
    collision_monitor_script = os.path.join(_ws_root, "scripts/runtime/dual_robot_collision_monitor.py")
    collision_args = ["python3", "-u", collision_monitor_script]
    if collision_output:
        collision_args += ["--output", collision_output]
    actions.append(
        TimerAction(
            period=3.5,  # right after MuJoCo comes up (T=3)
            actions=[
                ExecuteProcess(
                    cmd=collision_args,
                    name="dual_robot_collision_monitor",
                    output="screen",
                ),
            ],
        )
    )

    # ── Session reporter(s) ──
    # Per-robot reporter if session_duration_sec > 0 and output dir given.
    if session_duration_sec > 0 and session_output_dir:
        os.makedirs(session_output_dir, exist_ok=True)
        reporter_script = os.path.join(_ws_root, "scripts/bench/session_reporter.py")
        last_reporter = None
        for ns in ("robot_a", "robot_b"):
            out_path = os.path.join(session_output_dir, f"{ns}.json")
            proc = ExecuteProcess(
                cmd=[
                    "python3", "-u", reporter_script,
                    "--duration", str(session_duration_sec),
                    "--namespace", ns,
                    "--output", out_path,
                    "--scene-area-m2", str(scene_area_m2),
                ],
                name=f"session_reporter_{ns}",
                output="screen",
            )
            actions.append(TimerAction(period=nav_delay + 3.0, actions=[proc]))
            last_reporter = proc
        # Shut down the whole launch when the last reporter exits.
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=last_reporter,
                    on_exit=[
                        LogInfo(msg="[dual] session reporter exited — shutdown"),
                        Shutdown(reason="session complete"),
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument(
            "mujoco_model_path", default_value="",
            description="Path to MJCF scene. Defaults to demo3_dual.xml.",
        ),
        DeclareLaunchArgument("session_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("session_output_dir", default_value=""),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument(
            "collision_output_path",
            default_value="/tmp/dual_robot_collision_report.json",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
