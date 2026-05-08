"""Control-domain launch builders."""

from launch_ros.actions import Node

from .navigation import nav_profile_path


def build_wall_checker_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "wall_collision_checker"):
    params = [
        nav_profile_path("wall_checker.yaml"),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_safety",
        "executable": "wall_collision_checker.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_default_nav_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "default_nav",
):
    params = [
        nav_profile_path(profile),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_nav",
        "executable": "default_nav.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_goalpoint_bridge_node(ns: str | None, use_sim_time, remappings=None, name: str = "goalpoint_to_waypoint"):
    kwargs = {
        "package": "go2w_control",
        "executable": "goalpoint_to_waypoint.py",
        "name": name,
        "parameters": [{"use_sim_time": use_sim_time}],
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_frontier_recovery_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "frontier_recovery"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_control",
        "executable": "frontier_recovery.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_motion_monitor_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "motion_monitor"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_control",
        "executable": "motion_monitor.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_autonomy_enabler_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "autonomy_enabler"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_safety",
        "executable": "autonomy_enabler.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_gazebo_frontier_visual_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "gazebo_frontier_visual", condition=None):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_control",
        "executable": "gazebo_frontier_visual.py",
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
