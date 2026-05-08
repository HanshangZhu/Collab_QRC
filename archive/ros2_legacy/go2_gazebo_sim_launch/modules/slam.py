"""SLAM-domain launch builders."""

from launch_ros.actions import Node


def build_slam_odom_relay_node(
    ns: str | None,
    use_sim_time,
    *,
    input_topic: str,
    output_topic: str,
    gt_topic: str | None = None,
    output_frame_id: str = "world",
    output_child_frame_id: str = "base_link",
    bootstrap_from_gt: bool = False,
    require_gt_for_alignment: bool = False,
    name: str = "slam_odom_relay",
    condition=None,
):
    params = [
        {
            "use_sim_time": use_sim_time,
            "input_topic": input_topic,
            "output_topic": output_topic,
            "output_frame_id": output_frame_id,
            "output_child_frame_id": output_child_frame_id,
            "bootstrap_from_gt": bootstrap_from_gt,
            "require_gt_for_alignment": require_gt_for_alignment,
        }
    ]
    if gt_topic:
        params[0]["gt_topic"] = gt_topic

    kwargs = {
        "package": "go2w_perception",
        "executable": "slam_odom_relay.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)
