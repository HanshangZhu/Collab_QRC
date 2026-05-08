#!/usr/bin/env python3
"""Dual-Go2W MuJoCo launch for the door wedge & pass-through task (Phase 2 T1).

Single mujoco_ros2_control process loads two_rooms_door_scene.xml (both robots
embedded).  A shared controller_manager serves Robot A (un-prefixed joints) and
Robot B (b_-prefixed joints).  Each robot gets its own CHAMP + perception + nav
stack under /{ns}/.

Launch sequence:
  T=0    cleanup stale processes
  T=3    mujoco_ros2_control (ns=/mujoco_sim, combined URDF, door scene)
  T=5    sensor bridges (odom + contact per robot, door monitor)
  T=7    robot stacks (CHAMP, EKF, controller spawners, stand-up)
  T=?    readiness gate → perception + nav per robot
"""

from __future__ import annotations

import os
import shlex
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.dual_urdf import build_dual_mujoco_urdf, build_robot_b_urdf
from modules.orchestration import build_rviz_node
from modules.slam import build_slam_odom_relay_node
from go2_nav_algorithms.pipeline_components import (
    build_pointcloud_to_laserscan_node,
    build_simple_scan_mapper_cpp_node,
)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _build_cleanup_stale_command() -> str:
    patterns = [
        "mujoco_ros2_control",
        "/mujoco_sensor_bridge/",
        "/go2_nav_algorithms/",
        "/go2w_perception/",
        "/go2w_nav/",
        "/go2w_safety/",
        "/go2w_control/",
        "/go2w_spawn/",
        "/cfpa2_collaborative_autonomy/",
        "/champ_base/",
        "/robot_localization/",
        "/robot_state_publisher",
        "/controller_manager/spawner",
        "/pointcloud_to_laserscan_node",
        "/door_task/",
    ]
    command = [
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
        command.append(f"kill_pattern {shlex.quote(p)} TERM; ")
    command.append("sleep 1; ")
    for p in patterns:
        command.append(f"kill_pattern {shlex.quote(p)} KILL; ")
    command.append("sleep 0.5")
    return "".join(command)


def _launch_setup(context):
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    enable_navigation = _as_bool(_get(context, "enable_navigation"))
    # VLM controller is always on in this launch file — the legacy FSM
    # path has been deleted. These flags remain as True constants only so
    # existing conditions downstream read correctly (e.g. the odom bridge
    # suppresses its own TF because cartographer owns map→base_link).
    use_vlm_controller = True
    vlm_provider = _get(context, "vlm_provider")
    vlm_model = _get(context, "vlm_model")

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    champ_base_pkg = get_package_share_directory("champ_base")
    go2w_config_pkg = get_package_share_directory("go2w_config")

    mujoco_model_path = os.path.join(
        go2_gazebo_pkg, "mujoco", "two_rooms_door_scene.xml"
    )
    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "config", "ros_control", "ros_control_dual_mujoco_door.yaml"
    )
    # Sub-launch paths
    nav_launch = os.path.join(go2w_config_pkg, "launch", "navigation.launch.py")
    safety_launch = os.path.join(go2w_config_pkg, "launch", "safety.launch.py")
    obs_launch = os.path.join(go2w_config_pkg, "launch", "observability.launch.py")

    # ── URDF generation ──
    base_robot_description = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
    ).documentElement.toxml()

    # Combined URDF for mujoco_ros2_control (both robots, MujocoSystem plugin)
    combined_urdf = build_dual_mujoco_urdf(base_robot_description)

    # Per-robot URDFs for robot_state_publisher
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

    mujoco_plugin_dir = _find_mujoco_plugin_dir()

    sim_ns = "mujoco_sim"
    cm_path = f"/{sim_ns}/controller_manager"

    actions = [
        LogInfo(msg="[dual_go2w_mujoco_door] Starting door task launch"),
    ]

    # ── T=0: Cleanup ──
    if cleanup_stale:
        actions.append(
            ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_command()], output="screen")
        )

    # ── T=3: MuJoCo simulation node ──
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

    # ── T=5: Sensor bridges ──
    sensor_actions = []

    # Robot A odom bridge
    sensor_actions.append(
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace="robot_a",
            name="mujoco_odom_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mujoco_model_path},
                {"publish_rate": 50.0},
                {"base_body_name": "base_link"},
                {"odom_frame": "odom"},
                {"base_frame": "base_link"},
                # VLM path runs cartographer which owns map→base_link TF;
                # disable bridge TF to avoid dual-parent conflict on base_link.
                {"publish_tf": not use_vlm_controller},
                {"pose_topic": f"/{sim_ns}/base_link_site_pose_sensor/pose"},
                {"imu_topic": f"/{sim_ns}/imu_imu_sensor/imu"},
                {"republish_imu_topic": "imu/data"},
            ],
            remappings=[
                ("/tf", "/robot_a/tf"),
                ("/tf_static", "/robot_a/tf_static"),
            ],
            output="screen",
        )
    )

    # Robot B odom bridge
    sensor_actions.append(
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace="robot_b",
            name="mujoco_odom_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mujoco_model_path},
                {"publish_rate": 50.0},
                {"base_body_name": "b_base_link"},
                {"odom_frame": "odom"},
                {"base_frame": "b_base_link"},
                {"publish_tf": not use_vlm_controller},
                {"pose_topic": f"/{sim_ns}/b_base_link_site_pose_sensor/pose"},
                {"imu_topic": f"/{sim_ns}/b_imu_imu_sensor/imu"},
                {"republish_imu_topic": "imu/data"},
            ],
            remappings=[
                ("/tf", "/robot_b/tf"),
                ("/tf_static", "/robot_b/tf_static"),
            ],
            output="screen",
        )
    )

    # Robot A contact bridge
    sensor_actions.append(
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace="robot_a",
            name="mujoco_contact_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mujoco_model_path},
                {"publish_rate": 50.0},
                links_a,
            ],
            output="screen",
        )
    )

    # Robot B contact bridge
    sensor_actions.append(
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace="robot_b",
            name="mujoco_contact_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mujoco_model_path},
                {"publish_rate": 50.0},
                links_b,
            ],
            output="screen",
        )
    )

    actions.append(TimerAction(period=5.0, actions=sensor_actions))

    # ── T=7: Robot stacks (CHAMP + EKF + controller spawners + stand-up) ──
    # Stagger controller spawners to avoid service call collisions on the
    # shared controller_manager.  Robot A spawns first (T=7), Robot B at T=10.
    # Wheel spawners run after all joint/effort spawners have completed.
    robot_a_stack = build_dual_robot_stack(
        ns="robot_a",
        spawn_x="2.0",
        spawn_y="2.0",
        spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_a_urdf,
        joints_config=joints_a,
        links_config=links_a,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        joint_state_spawner_delay_sec=1.0,
        effort_spawner_delay_sec=2.0,
        standup_delay_sec=5.0,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2w",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="",  # No wheel controllers — pure legged mode for door task
        rsp_publish_frequency=100.0,
        use_mujoco=True,
        controller_manager_name=cm_path,
    )
    actions.append(TimerAction(period=7.0, actions=robot_a_stack))

    robot_b_stack = build_dual_robot_stack(
        ns="robot_b",
        spawn_x="6.0",
        spawn_y="2.0",
        spawn_yaw="3.14159",
        use_sim_time=use_sim_time,
        robot_description=robot_b_urdf,
        joints_config=joints_b,
        links_config=links_b,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        standup_delay_sec=5.0,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2w",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="",  # No wheel controllers — pure legged mode for door task
        rsp_publish_frequency=100.0,
        use_mujoco=True,
        controller_manager_name=cm_path,
    )
    actions.append(TimerAction(period=10.0, actions=robot_b_stack))

    # ── Door assist controller spawner ──
    # The door hinge is exposed via a third ros2_control block injected
    # by dual_urdf.py. A forward_command_controller writes effort to it;
    # door_lock_from_button_node publishes the effort target based on
    # whether the pressure pad is pressed.
    door_assist_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "door_assist_controller",
            "--controller-manager",
            cm_path,
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    actions.append(TimerAction(period=12.0, actions=[door_assist_spawner]))

    # ── Readiness gate ──
    wait_for_platform = Node(
        package="go2w_spawn",
        executable="wait_for_ready.py",
        name="wait_for_platform",
        parameters=[
            {
                "mode": "imu_stable",
                "imu_topic": "/robot_a/imu/data",
                "angular_velocity_threshold": 0.15,
                "stable_count": 5,
                "check_interval_sec": 1.0,
                "timeout_sec": 90.0,
                "gate_name": "dual_platform",
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    # ── Post-readiness: perception + nav per robot ──
    robot_actions = []

    lidar_topic_map = {
        "robot_a": f"/{sim_ns}/mujoco_lidar_sensor/registered_scan",
        "robot_b": f"/{sim_ns}/b_mujoco_lidar_sensor/registered_scan",
    }
    base_link_map = {"robot_a": "base_link", "robot_b": "b_base_link"}

    for ns in ("robot_a", "robot_b"):
        nav_odom_topic = f"/{ns}/odom/nav"
        planning_scan_topic = f"/{ns}/scan_3d"
        perception_cloud_topic = f"/{ns}/registered_scan_reliable"
        robot_base_link = base_link_map[ns]

        # Static TF: world → odom (identity) so nav can look up world→base_link
        robot_actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=ns,
                name="world_to_odom_tf",
                arguments=[
                    "--frame-id", "world",
                    "--child-frame-id", "odom",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                remappings=[
                    ("/tf_static", f"/{ns}/tf_static"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="log",
            )
        )

        # Twist bridge (TwistStamped -> Twist -> cmd_vel_legged)
        # Skip hybrid_cmd_router for the door task: the router publishes zeros
        # at 20 Hz on cmd_vel_legged even when idle, overriding the PUSH skill.
        # Instead, route nav cmd_vel directly to CHAMP's legged input.
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=ns,
                remappings=[
                    ("cmd_vel", "cmd_vel_legged"),
                ],
                output="screen",
            )
        )

        # QoS bridge
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="qos_bridge.py",
                namespace=ns,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": lidar_topic_map[ns]},
                    {"output_topic": perception_cloud_topic},
                ],
                output="screen",
            )
        )

        # Ground-truth odom relay
        robot_actions.append(
            build_slam_odom_relay_node(
                ns=ns,
                use_sim_time=use_sim_time,
                name="gt_odom_relay",
                input_topic=f"/{ns}/odom/ground_truth",
                output_topic=nav_odom_topic,
                output_frame_id="world",
                output_child_frame_id=robot_base_link,
            )
        )

        # Mapper:
        # - FSM path uses simple_scan_mapper_cpp (LaserScan → /{ns}/map) to
        #   feed astar_nav's A* planner.
        # - VLM path uses Cartographer per robot to emit /{ns}/map which
        #   vlm_controller renders as the SLAM occupancy panel.
        if not use_vlm_controller:
            robot_actions.append(
                build_pointcloud_to_laserscan_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    extra_params={
                        "target_frame": robot_base_link,
                        "min_height": 0.05,
                        "max_height": 0.60,
                        "range_min": 0.10,
                        "range_max": 8.0,
                    },
                    remappings=[
                        ("/tf", f"/{ns}/tf"),
                        ("/tf_static", f"/{ns}/tf_static"),
                        ("cloud_in", perception_cloud_topic),
                        ("scan", planning_scan_topic),
                    ],
                )
            )
            robot_actions.append(
                build_simple_scan_mapper_cpp_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    extra_params={
                        "scan_topic": planning_scan_topic,
                        "odom_topic": nav_odom_topic,
                        "map_topic": f"/{ns}/map",
                        "map_frame": "world",
                        "startup_delay": 0.0,
                        "max_scan_odom_dt": 0.25,
                        "max_range": 8.0,
                        "max_clear_distance": 3.5,
                        "update_rate": 4.0,
                        "miss_decrement": 2,
                        "decay_interval_sec": 2.0,
                        "decay_amount": 1,
                        # Door corridor exemption for RRT*: force cells free
                        # so the planner routes through the door opening.
                        "exempt_x_min": 3.5,
                        "exempt_x_max": 4.8,
                        "exempt_y_min": 1.4,
                        "exempt_y_max": 2.6,
                    },
                    remappings=[
                        ("/tf", f"/{ns}/tf"),
                        ("/tf_static", f"/{ns}/tf_static"),
                    ],
                )
            )
        else:
            # Cartographer 2D for VLM path. Each robot runs its own carto
            # instance in its own namespaced TF tree. Ground-truth odom is
            # fed in so the map frame is pinned to world (no scan-matching
            # drift in the long symmetric rooms). Robot B's URDF prefixes
            # frames with `b_`, so we materialize a tweaked lua into /tmp.
            vlm_pkg = get_package_share_directory("vlm_explorer")
            base_lua = os.path.join(vlm_pkg, "config", "cartographer_sim_2d.lua")
            if robot_base_link == "base_link":
                carto_cfg_dir = os.path.dirname(base_lua)
                carto_cfg_basename = "cartographer_sim_2d.lua"
            else:
                carto_cfg_dir = "/tmp"
                carto_cfg_basename = f"cartographer_sim_2d_{ns}.lua"
                with open(base_lua) as _f:
                    _lua = _f.read()
                _lua = _lua.replace(
                    'tracking_frame = "imu"',
                    'tracking_frame = "b_imu"',
                ).replace(
                    'published_frame = "base_link"',
                    'published_frame = "b_base_link"',
                )
                with open(os.path.join(carto_cfg_dir, carto_cfg_basename), "w") as _f:
                    _f.write(_lua)
            tf_remaps_carto = [
                ("/tf", f"/{ns}/tf"),
                ("/tf_static", f"/{ns}/tf_static"),
            ]
            robot_actions.append(
                Node(
                    package="cartographer_ros",
                    executable="cartographer_node",
                    namespace=ns,
                    name="cartographer_node",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "-configuration_directory", carto_cfg_dir,
                        "-configuration_basename", carto_cfg_basename,
                    ],
                    remappings=tf_remaps_carto + [
                        ("points2", perception_cloud_topic),
                        ("imu", f"/{ns}/imu/data"),
                    ],
                    output="log",
                )
            )
            robot_actions.append(
                Node(
                    package="cartographer_ros",
                    executable="cartographer_occupancy_grid_node",
                    namespace=ns,
                    name="cartographer_occupancy_grid_node",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "-resolution=0.05",
                        "-publish_period_sec=0.5",
                    ],
                    remappings=tf_remaps_carto + [("map", "map_prob")],
                    output="log",
                )
            )
            robot_actions.append(
                Node(
                    package="go2w_perception",
                    executable="probability_grid_binarizer.py",
                    namespace=ns,
                    name="probability_grid_binarizer",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": f"/{ns}/map_prob"},
                        {"output_topic": f"/{ns}/map"},
                        {"free_threshold": 49},
                        {"occupied_threshold": 65},
                        {"min_occupied_component_cells": 2},
                        {"fill_holes": True},
                        {"hole_neighbor_threshold": 7},
                    ],
                    remappings=tf_remaps_carto,
                    output="log",
                )
            )

        # Navigation sub-launch.
        # Skip it entirely for the LLM controller path: the nav planner
        # publishes cmd_vel_stamped at its control rate even while /stop=1
        # (its early-exit branches in !has_goal / startup paths still emit),
        # and that flow goes through twist_bridge → cmd_vel_legged and
        # overwrites the LLM's commands. With the nav sub-launch off, the
        # LLM controller is the only publisher on cmd_vel_legged.
        if enable_navigation and not use_vlm_controller:
            # Door task uses aggressive obstacle thresholds so the robot
            # drives close enough for bumper contact with the door panel.
            # Migrated from reactive_nav_node → astar_nav_node (2026-04-24).
            door_nav_config = os.path.join(
                go2w_config_pkg, "config", "nav", "astar_nav_door.yaml"
            )
            robot_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "robot_namespace": ns,
                        "use_sim_time": str(use_sim_time),
                        "map_frame": "world",
                        "remap_tf": "true",
                        "nav_backend": "astar",
                        "nav_config": door_nav_config,
                        "scan_topic": planning_scan_topic,
                        "odom_topic": nav_odom_topic,
                        "waypoint_input_suffix": "/way_point",
                        "cfpa2_goal_topic_suffix": "/way_point_exploration",
                    }.items(),
                )
            )

        # Safety sub-launch DISABLED for door task.
        # wall_collision_checker publishes stop=1 at 0.32m from walls,
        # which triggers the nav planner's external_stop and prevents
        # the robot from approaching the door closely enough for bumper
        # contact. autonomy_enabler is only needed for FAR, not astar.

        # Observability sub-launch
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(obs_launch),
                launch_arguments={
                    "robot_namespace": ns,
                    "use_sim_time": str(use_sim_time),
                    "experiment_name": "door_task",
                }.items(),
            )
        )

    # Wire readiness gate → robot actions
    # Start readiness gate at T=15 (after cleanup, MuJoCo, sensor bridges, robot stacks)
    if robot_actions:
        actions.append(TimerAction(period=15.0, actions=[wait_for_platform]))
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_platform,
                    on_exit=robot_actions,
                )
            )
        )

    # ── Door task nodes (T=5 for monitor, post-readiness for executor/coordinator) ──
    door_task_pkg = get_package_share_directory("door_task")
    door_task_config = os.path.join(door_task_pkg, "config", "door_task.yaml")

    # Door monitor + button monitor start with sensor bridges
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="door_task",
                    executable="door_monitor_node",
                    name="door_monitor",
                    parameters=[
                        door_task_config,
                        {"use_sim_time": use_sim_time},
                    ],
                    output="screen",
                ),
                Node(
                    package="door_task",
                    executable="button_monitor_node",
                    name="button_monitor",
                    parameters=[
                        door_task_config,
                        {"use_sim_time": use_sim_time},
                    ],
                    output="screen",
                ),
                Node(
                    package="door_task",
                    executable="door_lock_from_button_node",
                    name="door_lock_from_button",
                    parameters=[
                        door_task_config,
                        {"use_sim_time": use_sim_time},
                    ],
                    output="screen",
                ),
            ],
        )
    )

    # Door task control nodes start after readiness gate — VLM controller
    # (strategy) + perception node (YOLO + IoU tracker + CLIP + world_dict).
    door_task_actions = [
        Node(
            package="door_task",
            executable="vlm_controller_node",
            name="vlm_controller",
            parameters=[
                door_task_config,
                {"use_sim_time": use_sim_time},
                {"vlm_provider": vlm_provider},
                {"vlm_model": vlm_model},
            ],
            output="screen",
        ),
        Node(
            package="door_task",
            executable="perception_node",
            name="perception_node",
            parameters=[
                door_task_config,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
    ]

    # Attach door task actions to readiness gate (they run after robot stacks are ready)
    if door_task_actions:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_platform,
                    on_exit=door_task_actions,
                )
            )
        )

    # ── RViz ──
    if rviz:
        rviz_config = os.path.join(go2_gazebo_pkg, "rviz", "single_go2w_gazebo_cfpa2.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[build_rviz_node(rviz_config, use_sim_time, name="rviz2_door_task")],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("enable_navigation", default_value="true"),
            DeclareLaunchArgument("vlm_provider", default_value="xai"),
            DeclareLaunchArgument("vlm_model", default_value="grok-4-1-fast-non-reasoning"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
