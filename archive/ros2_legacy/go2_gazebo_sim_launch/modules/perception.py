"""Perception-domain launch builders."""

from launch_ros.actions import Node


def build_pointcloud_to_laserscan_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "pointcloud_to_laserscan"):
    params = [{
        "use_sim_time": use_sim_time,
        "target_frame": "base_link",
        "transform_tolerance": 1.0,
        "min_height": 0.3,
        "max_height": 1.0,
        "angle_min": -3.14159,
        "angle_max": 3.14159,
        "angle_increment": 0.006135923151543,
        "scan_time": 0.1,
        "range_min": 0.2,
        "range_max": 20.0,
        "use_inf": True,
    }]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "pointcloud_to_laserscan",
        "executable": "pointcloud_to_laserscan_node",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_qos_bridge_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "qos_bridge"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2w_perception",
        "executable": "qos_bridge.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)
