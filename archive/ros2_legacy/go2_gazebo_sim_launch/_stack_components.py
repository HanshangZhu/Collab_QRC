"""Compatibility shim for launch builders.

Prefer importing builders from `go2_gazebo_sim.launch.modules.*` directly.
This module is kept for one release cycle to preserve existing launch imports.
"""

from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.control import (
    build_autonomy_enabler_node,
    build_frontier_recovery_node,
    build_gazebo_frontier_visual_node,
    build_goalpoint_bridge_node,
    build_motion_monitor_node,
    build_default_nav_node,
    build_wall_checker_node,
)
from modules.navigation import (
    build_geometric_frontier_node,
    build_simple_scan_mapper_node,
    nav_algorithms_profile_path,
    nav_profile_path,
)
from modules.orchestration import build_rviz_node
from modules.perception import build_pointcloud_to_laserscan_node, build_qos_bridge_node
from modules.slam import build_slam_odom_relay_node

__all__ = [
    "nav_profile_path",
    "nav_algorithms_profile_path",
    "build_rviz_node",
    "build_wall_checker_node",
    "build_geometric_frontier_node",
    "build_simple_scan_mapper_node",
    "build_default_nav_node",
    "build_goalpoint_bridge_node",
    "build_frontier_recovery_node",
    "build_motion_monitor_node",
    "build_autonomy_enabler_node",
    "build_gazebo_frontier_visual_node",
    "build_pointcloud_to_laserscan_node",
    "build_qos_bridge_node",
    "build_slam_odom_relay_node",
    "build_namespaced_robot_description",
    "build_dual_robot_stack",
]
