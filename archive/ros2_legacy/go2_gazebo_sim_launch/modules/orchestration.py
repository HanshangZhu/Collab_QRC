"""Orchestration-domain launch builders."""

from launch_ros.actions import Node


def build_rviz_node(rviz_file: str, use_sim_time, condition=None, namespace: str | None = None, name: str = "rviz2"):
    kwargs = {
        "package": "rviz2",
        "executable": "rviz2",
        "arguments": ["-d", rviz_file],
        "parameters": [{"use_sim_time": use_sim_time}],
        "output": "screen",
        "name": name,
    }
    if namespace:
        kwargs["namespace"] = namespace
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)
