from pathlib import Path

import yaml


GO2W_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = GO2W_ROOT.parents[1]


def test_3d_costmap_overlay_uses_traversability_for_global_and_local_costmaps():
    overlay_path = (
        GO2W_ROOT
        / "go2w_config"
        / "config"
        / "nav"
        / "nav2_3d_costmap_overlay.yaml"
    )
    cfg = yaml.safe_load(overlay_path.read_text())

    global_params = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
    controller_params = cfg["controller_server"]["ros__parameters"]

    for params in (global_params, local_params):
        assert params["plugins"] == ["static_layer", "inflation_layer"]
        assert params["static_layer"]["map_topic"] == "/robot/traversability_grid"
    assert controller_params["goal_checker"]["xy_goal_tolerance"] == 0.20
    assert controller_params["goal_checker"]["stateful"] is False


def test_3d_explore_ramp_assist_holds_verified_goal_between_sparse_updates():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()

    assert '"goal_stale_sec": 8.0' in text


def test_3d_explore_ramp_goal_uses_monotonic_centerline_ascent():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()

    assert '"monotonic_ascent_enabled": True' in text
    assert '"monotonic_hold_sec": 30.0' in text
    assert '"monotonic_terminal_hold_enabled": True' in text
    assert '"ascent_terminal_x": 10.80' in text
    assert '"max_x": 11.2' in text


def test_3d_explore_keeps_ramp_assist_active_until_platform_entry():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()
    assist_block = text.split('name="ramp_cmd_vel_assist"')[1].split("remappings", 1)[0]

    assert '"max_x": 11.2' in assist_block


def test_fastlio_launch_passes_ramp_suppression_to_stuck_watchdog_not_path_relay():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    text = launch_path.read_text()
    path_relay_block = text.split('name=f"path_relay_{robot_ns}"')[0].rsplit(
        "path_relay_node = ExecuteProcess", 1
    )[1]
    watchdog_block = text.split('name=f"stuck_watchdog_{robot_ns}"')[0].rsplit(
        "stuck_watchdog_node = ExecuteProcess", 1
    )[1]

    assert "ramp_suppress_enabled" not in path_relay_block
    assert "ramp_suppress_enabled" in watchdog_block
    assert "STUCK_WATCHDOG_WINDOW_SEC" not in path_relay_block
    assert "STUCK_WATCHDOG_WINDOW_SEC" in watchdog_block
    assert "stuck_window_sec" in watchdog_block


def test_3d_explore_uses_long_stuck_window_for_ramp_demo():
    script_path = REPO_ROOT / "scripts" / "launch" / "nav_test_3d_explore.sh"
    text = script_path.read_text()

    assert 'STUCK_WATCHDOG_WINDOW_SEC="${STUCK_WATCHDOG_WINDOW_SEC:-9999.0}"' in text


def test_3d_explore_keeps_cfpa2_locked_through_platform_hold_zone():
    script_path = REPO_ROOT / "scripts" / "launch" / "nav_test_3d_explore.sh"
    text = script_path.read_text()

    assert 'params["ramp_ascent_lock_max_x"] = 13.0' in text
    assert 'params["ramp_ascent_lock_max_abs_y"] = 1.2' in text


def test_go2w_hybrid_router_force_wheel_covers_platform_entry_goal():
    cfg_path = (
        GO2W_ROOT
        / "go2w_config"
        / "config"
        / "control"
        / "go2w_hybrid_motion.yaml"
    )
    text = cfg_path.read_text()

    assert "ramp_force_max_goal_x: 11.2" in text


def test_fastlio_rviz_process_sanitizes_snap_environment():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    text = launch_path.read_text()
    rviz_block = text.split('name="rviz2_nav_test"')[1].split("output=", 1)[0]

    assert "additional_env=_rviz_clean_env()" in rviz_block
    assert '"SNAP": ""' in text
    assert '"GTK_PATH": ""' in text
    assert '"XDG_DATA_HOME": ""' in text
