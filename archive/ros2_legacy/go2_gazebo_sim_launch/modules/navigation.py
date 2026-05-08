"""Navigation-domain launch builders."""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def nav_profile_path(filename: str) -> str:
    return os.path.join(get_package_share_directory("go2w_config"), "config", "nav", filename)


def nav_algorithms_profile_path(filename: str) -> str:
    return os.path.join(get_package_share_directory("go2_nav_algorithms"), "config", "nav", filename)


def build_geometric_frontier_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "geometric_frontier",
    condition=None,
):
    params = [
        nav_algorithms_profile_path(profile),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_nav_algorithms",
        "executable": "simple_frontier_explorer.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_simple_scan_mapper_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "simple_scan_mapper",
    executable: str = "simple_scan_mapper_cpp",
    condition=None,
):
    params = [
        nav_algorithms_profile_path(profile),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_nav_algorithms",
        "executable": executable,
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)
