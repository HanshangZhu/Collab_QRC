#!/usr/bin/env python3
"""Bring up Nav2 full stack (SmacHybrid + DWB) for robot_a.

Standalone launch — use ALONGSIDE an existing sim that publishes:
  /robot_a/map        (OccupancyGrid, e.g. from octomap_server)
  /robot_a/odom/nav   (Odometry, used as nav2 odom topic)
  /robot_a/scan_3d    (LaserScan, for local costmap obstacles)
  /robot_a/tf         (namespaced TF tree)

Nav2's controller publishes /cmd_vel which we redirect to
/robot_a/cmd_vel — compatible with the existing twist_bridge → router
chain.

Usage:
  ros2 launch go2_gazebo_sim nav2_robot_a.launch.py

Then send a goal in RViz (2D Goal Pose) on topic /goal_pose, OR via
action client on /robot_a/navigate_to_pose.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    pkg_config = get_package_share_directory("go2w_config")
    default_params_file = os.path.join(
        pkg_config, "config", "nav", "nav2_go2w_full_stack.yaml"
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="robot_a",
        description="Namespace under which nav2 nodes run (cmd_vel etc.)"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file", default_value=default_params_file,
        description="Full stack nav2 params YAML"
    )

    # TF remap: every nav2 server has to read namespaced TF.
    tf_remaps = [
        ("/tf", ["/", namespace, "/tf"]),
        ("/tf_static", ["/", namespace, "/tf_static"]),
    ]

    # Rewrite the yaml so ros2_param node-section names get the namespace
    # prefix. Without this, plain top-level sections like `local_costmap:`
    # don't match the actual node path `/robot_a/local_costmap/local_costmap`
    # and parameters silently fail to load (we'd see e.g. robot_radius
    # default to 0 inside the costmap). nav2_common.RewrittenYaml is the
    # standard nav2_bringup approach for namespaced multi-robot setups.
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )
    # Build per-node parameter lists. Use the RewrittenYaml output for
    # namespace-aware param matching.
    common_params = [rewritten_params]

    nav2_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),

            # ── controller_server ─────────────────────────────────────
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_params,
                remappings=tf_remaps + [
                    # DWB writes /cmd_vel; existing twist_bridge consumes
                    # /robot_a/cmd_vel (PushRosNamespace makes this absolute).
                    # Local-costmap subscribes /scan internally — nav2 yaml
                    # already routes it to /robot_a/scan_3d explicitly.
                ],
            ),

            # ── planner_server ────────────────────────────────────────
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_params,
                remappings=tf_remaps,
            ),

            # ── behavior_server (recovery primitives) ─────────────────
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common_params,
                remappings=tf_remaps,
            ),

            # ── bt_navigator (orchestrates planner + controller) ──────
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common_params,
                remappings=tf_remaps,
            ),

            # ── lifecycle_manager (auto-activates everything) ─────────
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=common_params,
                # No tf_remaps — lifecycle_manager doesn't read TF.
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_params_file,
            nav2_nodes,
        ]
    )
