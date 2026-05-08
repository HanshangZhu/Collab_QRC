#!/usr/bin/env python3
"""Shared navigation sub-launch: mapper + CFPA2 frontier + local planner.

Included by both sim and real top-level launch files with platform-appropriate args.

nav_backend:
  default    — default_nav.py (Python A* grid planner with D* Lite + recovery)
  astar      — astar_nav_node (C++ A* + pure-pursuit + oriented footprint check)
  far        — CMU autonomy stack: terrain_analysis + far_planner + localPlanner/pathFollower
  nav2_mppi  — Nav2 SmacPlannerHybrid + MPPI + behavior_server +
               lifecycle_manager, polygon footprint, fast_lio_tf_adapter
               for TF, stuck_watchdog for self-recovery, cfpa2_to_nav2_bridge
               for goal forwarding. Production stack on real robot since
               2026-04-29. Default for both real_autonomy.sh entry points.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is
    launched in a namespace (``/robot/far_planner``).  This helper
    strips the outer node-name key and returns just the parameter dict
    so it can be merged into the launch ``parameters`` list directly.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    # Expect { node_name: { ros__parameters: { ... } } }
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    # Fallback: return everything flat
    return data


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _setup(context):
    robot_ns = _get(context, "robot_namespace")
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    map_frame = _get(context, "map_frame")
    remap_tf = _as_bool(_get(context, "remap_tf"))
    nav_backend = _get(context, "nav_backend").strip().lower() or "default"
    # Back-compat alias: old invocations still pass "reactive" — accept it
    # as synonymous with the Python default_nav. "rrt_star" and "far_rrt_star"
    # are GONE (reactive_nav_node deleted); we silently upgrade them to
    # "astar" so legacy launches keep working.
    if nav_backend == "reactive":
        nav_backend = "default"
    elif nav_backend in ("rrt_star", "far_rrt_star"):
        nav_backend = "astar"

    scan_topic = _get(context, "scan_topic") or f"/{robot_ns}/scan_3d"
    odom_topic = _get(context, "odom_topic") or f"/{robot_ns}/odom/nav"
    map_topic = _get(context, "map_topic") or f"/{robot_ns}/map"
    waypoint_suffix = _get(context, "waypoint_input_suffix") or "/way_point_coord"

    # 3D point cloud topic for FAR terrain analysis (must be in map frame)
    registered_scan_topic = _get(context, "registered_scan_topic").strip() or f"/{robot_ns}/registered_scan_reliable"

    cfpa2_w_ig = float(_get(context, "cfpa2_w_ig"))
    cfpa2_w_c = float(_get(context, "cfpa2_w_c"))
    cfpa2_w_momentum = float(_get(context, "cfpa2_w_momentum"))
    cfpa2_min_utility = float(_get(context, "cfpa2_min_utility"))
    cfpa2_goal_topic_suffix = _get(context, "cfpa2_goal_topic_suffix").strip() or "/way_point_coord"
    max_linear_speed_str = _get(context, "max_linear_speed").strip()
    require_settle_str = _get(context, "require_settle_before_motion").strip()
    nav_map_topic_str = _get(context, "nav_map_topic").strip()
    switch_hysteresis_str = _get(context, "cfpa2_switch_hysteresis").strip()

    # FAR-specific params
    far_max_speed = float(_get(context, "far_max_speed"))
    far_robot_id = int(_get(context, "far_robot_id"))

    nav_config = _get(context, "nav_config")
    cfpa2_config = _get(context, "cfpa2_config")

    tf_remaps = []
    if remap_tf:
        tf_remaps = [
            ("/tf", f"/{robot_ns}/tf"),
            ("/tf_static", f"/{robot_ns}/tf_static"),
        ]

    # Default log level: only CFPA2 frontier and nav planner at INFO;
    # everything else at WARN to reduce console noise.
    log_warn = ["--ros-args", "--log-level", "warn"]
    log_info = ["--ros-args", "--log-level", "info"]

    actions = []

    # ── CFPA2 Frontier ──
    cfpa2_params = {
        "use_sim_time": use_sim_time,
        "robot_namespace": robot_ns,
        "namespaces": [robot_ns],
        "goal_topic_suffix": cfpa2_goal_topic_suffix,
        "marker_frame_override": map_frame,
        "cfpa2_w_ig": cfpa2_w_ig,
        "cfpa2_w_c": cfpa2_w_c,
        "cfpa2_w_momentum": cfpa2_w_momentum,
        "cfpa2_min_utility": cfpa2_min_utility,
    }
    if switch_hysteresis_str:
        cfpa2_params["switch_hysteresis"] = float(switch_hysteresis_str)

    explore_flag = _as_bool(_get(context, "explore"))
    if explore_flag:
        actions.append(
            Node(
                package="cfpa2_collaborative_autonomy",
                executable="cfpa2_single_robot_node",
                name="cfpa2_single_robot",
                parameters=[cfpa2_config, cfpa2_params],
                ros_arguments=log_info,
                output="screen",
            )
        )

    # ── Local Planner Backend ──
    if nav_backend == "nav2_mppi":
        # Nav2 stack with SmacPlannerHybrid (REEDS_SHEPP) + MPPIController
        # + behavior_server (Spin/BackUp/Wait) + bt_navigator + lifecycle_manager.
        # Augmented by:
        #   - fast_lio_tf_adapter (publishes /<ns>/odom/nav + odom→base_link TF
        #     from Fast-LIO; replaces slam_odom_relay; works on real with
        #     bootstrap_from_gt:=false)
        #   - cfpa2_to_nav2_bridge (CFPA2 way_point_coord → Nav2 goal_pose)
        #   - path_relay (Nav2 /plan → /planned_path for legacy RViz config)
        #   - stuck_watchdog (10s no-motion → Nav2 BackUp + republish goal)
        # Per-platform yaml: Go2W → nav2_go2w_full_stack.yaml,
        # Go2 → nav2_go2_full_stack.yaml.
        from launch.actions import ExecuteProcess
        from launch_ros.actions import PushRosNamespace
        from launch.actions import GroupAction
        from nav2_common.launch import RewrittenYaml

        # Determine platform from robot_namespace heuristic + base_frame param.
        # Real launches pass base_frame via robot_model arg; for navigation.launch.py
        # we read a lightweight `nav2_yaml_filename` override or fall back to
        # robot_model="go2w" assumption. Currently real_autonomy.sh always
        # invokes with go2w paths, so default to wheeled yaml.
        _robot_model = (_get(context, "robot_model") or "").strip().lower()
        _is_legged_only = _robot_model == "go2"
        # Optional override: real-robot launches pass nav2_yaml_override to
        # select a real-tuned yaml (e.g. nav2_go2_real.yaml) with reduced
        # SmacPlanner iteration cap, smaller MPPI batch, softer obstacle
        # critic, etc. Sim launches leave it empty and get the auto-pick.
        _yaml_override = _get(context, "nav2_yaml_override").strip()
        # Optional second-layer override for small profile deltas on top of the
        # base yaml (e.g. holonomic Go2W Nav2 profile while preserving the
        # default diff-drive real config file unchanged).
        _yaml_extra_override = _get(context, "nav2_yaml_extra_override").strip()
        if _yaml_override:
            _nav2_yaml_name = _yaml_override
        else:
            _nav2_yaml_name = (
                "nav2_go2_full_stack.yaml" if _is_legged_only
                else "nav2_go2w_full_stack.yaml"
            )
        _go2w_config_pkg = get_package_share_directory("go2w_config")
        _nav2_yaml_path = os.path.join(
            _go2w_config_pkg, "config", "nav", _nav2_yaml_name
        )
        _base_frame = "b_base_link" if _is_legged_only else "base_link"

        # The yamls bake in /robot_a/ (go2w yaml) and /robot_b/ (go2 yaml) for
        # the dual-robot sim case. On real (ns=robot), those topics don't
        # exist — Nav2 silently subscribes to dead topics:
        #   global_costmap.static_layer.map_topic    /robot_a/map
        #   local_costmap.obstacle_layer.scan.topic  /robot_a/scan_3d
        #   controller_server.odom_topic             /robot_a/odom/nav
        # No map → planner cannot plan → no path ever appears in RViz, even
        # though TF and QoS are correct. RewrittenYaml.param_rewrites matches
        # by key name only and would rewrite every "topic"/"map_topic" in the
        # file (some are intentionally relative). Safer: do the namespace
        # substitution on the raw yaml content, drop it in a temp file.
        import tempfile, re
        with open(_nav2_yaml_path) as _f:
            _yaml_text = _f.read()
        _yaml_text = re.sub(r"/robot_[ab]/", f"/{robot_ns}/", _yaml_text)
        _tmp_yaml = tempfile.NamedTemporaryFile(
            mode="w", suffix=f"_{robot_ns}_nav2.yaml", delete=False
        )
        _tmp_yaml.write(_yaml_text)
        _tmp_yaml.close()
        _rewritten = RewrittenYaml(
            source_file=_tmp_yaml.name,
            root_key=robot_ns,
            param_rewrites={"use_sim_time": str(use_sim_time).lower()},
            convert_types=True,
        )
        _nav2_parameters = [_rewritten]
        if _yaml_extra_override:
            _overlay_path = os.path.join(
                _go2w_config_pkg, "config", "nav", _yaml_extra_override
            )
            _overlay_rewritten = RewrittenYaml(
                source_file=_overlay_path,
                root_key=robot_ns,
                param_rewrites={"use_sim_time": str(use_sim_time).lower()},
                convert_types=True,
            )
            _nav2_parameters.append(_overlay_rewritten)

        # Nav2 publishes plain Twist on `cmd_vel` (relative → /<ns>/cmd_vel
        # under PushRosNamespace). The real-bringup cmd chain expects the
        # downstream "auto" feed at /<ns>/cmd_vel_auto (consumed by
        # cmd_vel_activity_mux which arbitrates manual vs auto). The legacy
        # twist_bridge that connects /<ns>/cmd_vel_stamped → /<ns>/cmd_vel_auto
        # is for default_nav.py / astar (TwistStamped publishers) and is
        # completely silent under Nav2 (different topic name + message type).
        # Remap controller_server and behavior_server's cmd_vel directly to
        # cmd_vel_auto so the mux receives Nav2's output. Without this,
        # /<ns>/cmd_vel has 5 pubs / 0 subs while /<ns>/cmd_vel_auto has 0
        # pubs / 1 sub — robot receives /api/sport/request {x:0,y:0,z:0}
        # at heartbeat rate and stands still.
        cmd_vel_remap = [("cmd_vel", "cmd_vel_auto")]
        nav2_inner_nodes = [
            PushRosNamespace(robot_ns),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server",
                parameters=_nav2_parameters,
                remappings=(tf_remaps if remap_tf else []) + cmd_vel_remap,
                output="screen",
            ),
            Node(
                package="nav2_planner", executable="planner_server",
                name="planner_server",
                parameters=_nav2_parameters,
                remappings=tf_remaps if remap_tf else [],
                output="screen",
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server",
                parameters=_nav2_parameters,
                # spin/backup/wait recovery primitives also publish cmd_vel —
                # remap them too so recoveries actually drive the robot.
                remappings=(tf_remaps if remap_tf else []) + cmd_vel_remap,
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator",
                name="bt_navigator",
                parameters=_nav2_parameters,
                remappings=tf_remaps if remap_tf else [],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=_nav2_parameters,
                output="screen",
            ),
        ]
        actions.append(GroupAction(actions=nav2_inner_nodes))

        # Helper-script paths under repo's scripts/runtime/ — same on
        # sim and real (mounted by source build, not in install/).
        # Resolved relative to this launch file's REAL path (realpath follows
        # the colcon --symlink-install symlink back to src/). Works regardless
        # of the developer's home layout. Override with COLLAB_QRC_RUNTIME_DIR.
        _repo_scripts = os.environ.get(
            "COLLAB_QRC_RUNTIME_DIR",
            os.path.abspath(os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "..", "..", "..", "..", "scripts", "runtime",
            )),
        )

        # fast_lio_tf_adapter: publishes /<ns>/odom/nav + odom→base_link
        # TF from Fast-LIO. On real, no GT to bootstrap against → set
        # bootstrap_from_gt=false; the map frame's origin then equals
        # robot's spawn pose (whatever Fast-LIO set as origin at startup).
        adapter_cmd = [
            "python3", "-u",
            os.path.join(_repo_scripts, "fast_lio_tf_adapter.py"),
            "--ros-args",
            "-p", f"namespace:={robot_ns}",
            "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            # Real launches override to "/Odometry" because Fast-LIO is
            # launched un-namespaced; sim dual leaves the default
            # "Odometry" so the adapter prepends /<ns>/ correctly.
            "-p", f"input_topic:={_get(context, 'fast_lio_input_topic') or 'Odometry'}",
            "-p", "output_topic:=odom/nav",
            "-p", "output_frame_id:=odom",
            "-p", f"output_child_frame_id:={_base_frame}",
            # Sim dual: adapter is the SOLE owner of base_link's TF. Real:
            # the existing map→camera_init→body→base_link chain (slam.launch.py)
            # already parents base_link, plus a map→odom static identity makes
            # odom resolvable via tree walk. Adapter TF here would multi-parent.
            "-p", f"publish_tf:={_get(context, 'fast_lio_publish_tf') or 'true'}",
            # Bootstrap is sim-only privilege; real has no GT topic.
            "-p", f"bootstrap_from_gt:={'true' if use_sim_time else 'false'}",
            "-p", "gt_topic:=odom/ground_truth",
            "-p", "corrected_topic:=corrected_odom",
        ]
        # Only namespace /tf when the rest of the stack does (sim dual). On
        # real single-robot, every other publisher (static_transform_publisher,
        # Cartographer, …) writes to global /tf — namespacing the adapter's
        # output here would orphan it (1 pub, 0 subs), and Nav2 would
        # report "Invalid frame ID 'odom'" forever despite the adapter
        # cheerfully relaying messages.
        if remap_tf:
            adapter_cmd += [
                "-r", f"/tf:=/{robot_ns}/tf",
                "-r", f"/tf_static:=/{robot_ns}/tf_static",
            ]
        actions.append(
            ExecuteProcess(
                cmd=adapter_cmd,
                name=f"fast_lio_tf_adapter_{robot_ns}",
                output="screen",
            )
        )

        # cfpa2_to_nav2_bridge: way_point_coord → goal_pose for bt_navigator.
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_repo_scripts, "cfpa2_to_nav2_bridge.py"),
                    "--ros-args",
                    "-p", f"namespace:={robot_ns}",
                    "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                    "-p", "waypoint_topic:=way_point_coord",
                ],
                name=f"cfpa2_to_nav2_bridge_{robot_ns}",
                output="screen",
            )
        )

        # path_relay: Nav2 /plan → /planned_path for legacy RViz config.
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_repo_scripts, "path_relay.py"),
                    "--ros-args",
                    "-p", f"namespace:={robot_ns}",
                    "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                name=f"path_relay_{robot_ns}",
                output="screen",
            )
        )

        # robot_pose_marker: synthesizes /<ns>/robot_pose_marker (red triangle)
        # from /<ns>/odom/nav. Nav2 MPPI doesn't publish this marker (only the
        # legacy A*/default backends do); without this node the RViz config's
        # RobotPoseTriangle stays blank.
        # Footprint dims here are chosen by robot_model: Go2W is 0.70×0.35,
        # Go2 is 0.65×0.30 — matches the polygon footprints in the nav2 yamls.
        _length = "0.65" if _is_legged_only else "0.70"
        _width = "0.30" if _is_legged_only else "0.35"
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_repo_scripts, "robot_pose_marker.py"),
                    "--ros-args",
                    "-p", f"namespace:={robot_ns}",
                    "-p", "frame_id:=map",
                    "-p", f"length:={_length}",
                    "-p", f"width:={_width}",
                    "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                name=f"robot_pose_marker_{robot_ns}",
                output="screen",
            )
        )

        # stuck_watchdog: outer-loop self-recovery (10s no-motion → BackUp).
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_repo_scripts, "stuck_watchdog.py"),
                    "--ros-args",
                    "-p", f"namespace:={robot_ns}",
                    "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                name=f"stuck_watchdog_{robot_ns}",
                output="screen",
            )
        )
        return actions

    if nav_backend == "far":
        far_goal_topic_override = _get(context, "far_goal_topic").strip()
        far_way_point_out_override = _get(context, "far_way_point_out").strip()
        actions.extend(
            _build_far_stack(
                robot_ns=robot_ns,
                use_sim_time=use_sim_time,
                map_frame=map_frame,
                odom_topic=odom_topic,
                registered_scan_topic=registered_scan_topic,
                waypoint_suffix=waypoint_suffix,
                tf_remaps=tf_remaps,
                max_speed=far_max_speed,
                robot_id=far_robot_id,
                far_goal_topic_override=far_goal_topic_override,
                far_way_point_out_override=far_way_point_out_override,
            )
        )
        # FAR doesn't speak nav_status/v1 natively — adapter bridges its
        # /far_reach_goal_status + /way_point outputs into the canonical
        # topic so CFPA2 can fast-blacklist unreachable goals regardless
        # of which planner is active.
        actions.append(_far_status_adapter_node(robot_ns, use_sim_time, odom_topic, waypoint_suffix))
    else:
        # Shared remappings for astar_nav_node and default_nav.py
        nav_remappings = [
            ("/way_point", f"/{robot_ns}{waypoint_suffix}"),
            ("/odom/ground_truth", odom_topic),
            ("/scan", scan_topic),
            ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{robot_ns}/nav_status"),
            ("/planned_path", f"/{robot_ns}/planned_path"),
            ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
            ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
            ("/robot_pose_marker", f"/{robot_ns}/robot_pose_marker"),
        ]

        nav_extra = {
            "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
            "stop_topic": f"/{robot_ns}/stop",
        }

        if nav_backend == "astar":
            # C++ A* + pure-pursuit + oriented footprint validation.
            if max_linear_speed_str:
                nav_extra["max_linear_speed"] = float(max_linear_speed_str)
            nav_extra["map_frame"] = map_frame
            nav_extra["map_topic"] = f"/{robot_ns}/map"

            actions.append(
                Node(
                    package="go2w_nav",
                    executable="astar_nav_node",
                    namespace=robot_ns,
                    name="astar_nav",
                    parameters=[nav_config, {"use_sim_time": use_sim_time}, nav_extra],
                    remappings=nav_remappings + (tf_remaps if remap_tf else []),
                    ros_arguments=log_info,
                    output="screen",
                )
            )
        else:
            # default_nav — Python A*/D* Lite with recovery (legacy stable).
            if max_linear_speed_str:
                nav_extra["max_linear_speed"] = float(max_linear_speed_str)
            if require_settle_str:
                nav_extra["require_settle_before_motion"] = _as_bool(require_settle_str)
            if nav_map_topic_str:
                nav_extra["map_topic"] = nav_map_topic_str

            actions.append(
                Node(
                    package="go2w_nav",
                    executable="default_nav.py",
                    namespace=robot_ns,
                    name="default_nav",
                    parameters=[nav_config, {"use_sim_time": use_sim_time}, nav_extra],
                    remappings=nav_remappings,
                    ros_arguments=log_warn,
                    output="screen",
                )
            )

    return actions


def _far_status_adapter_node(robot_ns: str, use_sim_time: bool, odom_topic: str, waypoint_suffix: str):
    """Spawn far_status_adapter.py — bridges FAR's native outputs into
    nav_status/v1 JSON so CFPA2 can fast-blacklist unreachable goals.
    See docs/claude/nav_status_contract.md.
    """
    return Node(
        package="go2w_nav",
        executable="far_status_adapter.py",
        namespace=robot_ns,
        name="far_status_adapter",
        parameters=[{
            "use_sim_time": use_sim_time,
            "way_point_timeout_sec": 2.0,
            "unreachable_timeout_sec": 3.0,
            "far_heartbeat_timeout_sec": 5.0,
            "publish_rate_hz": 5.0,
        }],
        remappings=[
            ("/nav_status", f"/{robot_ns}/nav_status"),
            ("/far_reach_goal_status", f"/{robot_ns}/far_reach_goal_status"),
            ("/goal_point", f"/{robot_ns}{waypoint_suffix}"),
            ("/way_point", f"/{robot_ns}/way_point"),
            ("/far_planning_time", f"/{robot_ns}/far_planning_time"),
            ("/odom/ground_truth", odom_topic),
        ],
        output="screen",
    )


def _build_far_stack(
    *,
    robot_ns: str,
    use_sim_time: bool,
    map_frame: str,
    odom_topic: str,
    registered_scan_topic: str,
    waypoint_suffix: str,
    tf_remaps: list,
    max_speed: float,
    robot_id: int,
    far_goal_topic_override: str = "",
    far_way_point_out_override: str = "",
) -> list:
    """Build the CMU autonomy FAR planner stack with full namespacing.

    Topic flow:
      CFPA2 → /{ns}{waypoint_suffix} (goal_point)
        → far_planner → /{ns}/way_point (intermediate route waypoints)
          → localPlanner → pathFollower → /{ns}/cmd_vel_stamped

    Terrain pipeline:
      /{ns}/registered_scan_map (map-frame 3D cloud, provided by caller)
        → sensorScanGeneration → /{ns}/sensor_scan
        → terrainAnalysis       → /{ns}/terrain_map
        → terrainAnalysisExt    → /{ns}/terrain_map_ext
    """
    ns = robot_ns
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    nodes = []

    # ── Terrain analysis pipeline ──
    # These nodes assume the input point cloud is in map frame (no TF lookups).
    nodes.append(
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            namespace=ns,
            name="sensor_scan_generation",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ] + tf_remaps,
            output="screen",
        )
    )

    nodes.append(
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace=ns,
            name="terrain_analysis",
            parameters=[{
                "use_sim_time": use_sim_time,
                # Raise from 0.2 to 0.8 so terrain cloud includes full
                # wall height for FAR contour detection.  localPlanner
                # has its own relZ filter (maxRelZ=0.25) so it won't
                # be flooded by the extra points.
                "maxRelZ": 0.8,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        )
    )

    nodes.append(
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            namespace=ns,
            name="terrain_analysis_ext",
            parameters=[{
                "use_sim_time": use_sim_time,
                "maxRelZ": 0.8,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        )
    )

    # ── FAR planner (global route planner) ──
    # Reads goal_point from CFPA2/VLM mux, outputs intermediate way_point for localPlanner.
    # Has its own TF lookups — handles frame transforms internally via world_frame param.
    nodes.append(
        Node(
            package="far_planner",
            executable="far_planner",
            namespace=ns,
            name="far_planner",
            parameters=[
                # Load YAML params stripped of the node-name key so they
                # apply correctly under any ROS2 namespace.
                _load_yaml_params(os.path.join(far_pkg, "config", "default.yaml")),
                {
                    "use_sim_time": use_sim_time,
                    "world_frame": map_frame,
                    "graph_msger/robot_id": robot_id,
                    # Converge distance must exceed pathFollower stopDisThre (0.2m)
                    # so FAR declares waypoint reached before pathFollower stalls.
                    "g_planner/converge_distance": 0.5,
                    # terrain_free_Z: points with intensity (height above local
                    # ground) below this are "free", above are obstacles for
                    # V-Graph contour detection.  With corrected L1 LiDAR FOV
                    # (-15° to +42°) terrain_analysis now sees real wall height,
                    # but intensity mean ≈ 0.32 with 0.30 threshold caused ~50%
                    # obstacle classification → too many false contour polygons
                    # blocking all V-Graph edges.  0.45 keeps only real walls.
                    "util/terrain_free_Z": 0.45,
                    # obs_inflate_size: voxels to inflate obstacle contours (CMU default=1).
                    "util/obs_inflate_size": 1,
                },
            ],
            remappings=[
                ("/odom_world", odom_topic),
                ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
                ("/scan_cloud", f"/{ns}/terrain_map"),
                ("/terrain_local_cloud", registered_scan_topic),
                # Optional unwire: empty far_goal_topic_override leaves FAR
                # listening on its normal /{ns}{waypoint_suffix} input.
                # Non-empty redirects it (including "" to a topic with no
                # publisher, effectively idling FAR).
                ("/goal_point",
                 far_goal_topic_override if far_goal_topic_override
                 else f"/{ns}{waypoint_suffix}"),
                # Same pattern on the output side — redirect to a dead sink
                # when a TARE-direct pipeline owns /{ns}/way_point.
                ("/way_point",
                 far_way_point_out_override if far_way_point_out_override
                 else f"/{ns}/way_point"),
                ("/joy", f"/{ns}/joy"),
                ("/navigation_boundary", f"/{ns}/navigation_boundary"),
                ("/runtime", f"/{ns}/far_runtime"),
                ("/planning_time", f"/{ns}/far_planning_time"),
                # Per-robot graph exchange (use shared bus for multi-robot coordination)
                ("/robot_vgraph", f"/{ns}/robot_vgraph"),
                ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── localPlanner (local trajectory planner) ──
    # No TF lookups — assumes point cloud and odometry in same frame.
    # Reads way_point from FAR, generates local path for pathFollower.
    nodes.append(
        Node(
            package="local_planner",
            executable="localPlanner",
            namespace=ns,
            name="localPlanner",
            parameters=[{
                "use_sim_time": use_sim_time,
                "pathFolder": os.path.join(local_planner_pkg, "paths"),
                "vehicleLength": 0.3,
                "vehicleWidth": 0.7,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": False,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                "checkRotObstacle": False,
                "adjacentRange": 3.0,
                # With corrected L1 LiDAR FOV, terrain_analysis produces
                # wider intensity range.  0.50 ensures only real walls
                # block localPlanner paths.  Must be > terrain_free_Z (0.45).
                "obstacleHeightThre": 0.50,
                "groundHeightThre": 0.1,
                "costHeightThre": 0.1,
                "costScore": 0.02,
                "useCost": False,
                "pointPerPathThre": 2,
                "minRelZ": -0.5,
                "maxRelZ": 0.25,
                "maxSpeed": max_speed,
                "dirWeight": 0.02,
                "dirThre": 90.0,
                "dirToVehicle": False,
                "pathScale": 0.75,
                "minPathScale": 0.5,
                "pathScaleStep": 0.25,
                "pathScaleBySpeed": True,
                "minPathRange": 1.0,
                "pathRangeStep": 0.5,
                "pathRangeBySpeed": True,
                "pathCropByGoal": True,
                "autonomyMode": True,
                "autonomySpeed": max_speed,
                "joyToSpeedDelay": 2.0,
                "joyToCheckObstacleDelay": 5.0,
                "goalClearRange": 0.5,
                "goalX": 0.0,
                "goalY": 0.0,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/way_point", f"/{ns}/way_point"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/overall_map", f"/{ns}/terrain_map"),
                ("/joy", f"/{ns}/joy"),
                ("/path", f"/{ns}/local_path"),
                ("/freePaths", f"/{ns}/free_paths"),
            ],
            output="screen",
        )
    )

    # ── pathFollower (pure-pursuit path execution) ──
    # Publishes TwistStamped on cmd_vel (remapped to /{ns}/cmd_vel_stamped).
    nodes.append(
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
                "twoWayDrive": False,
                "lookAheadDis": 0.5,
                "yawRateGain": 1.5,
                "stopYawRateGain": 1.5,
                "maxYawRate": 80.0,
                "maxSpeed": max_speed,
                "maxAccel": 2.0,
                "switchTimeThre": 1.0,
                "dirDiffThre": 0.4,
                "omniDirDiffThre": 1.5,
                "noRotSpeed": 10.0,
                # CMU default 0.3 too large when localPlanner produces
                # short 0.23m paths at minimum scale.
                "stopDisThre": 0.15,
                "slowDwnDisThre": 0.75,
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
                "autonomySpeed": max_speed,
                "joyToSpeedDelay": 2.0,
                "goalCloseDis": 0.4,
                "is_real_robot": False,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/path", f"/{ns}/local_path"),
                ("/cmd_vel", f"/{ns}/cmd_vel_stamped"),
                ("/joy", f"/{ns}/joy"),
                ("/speed", f"/{ns}/speed"),
                ("/stop", f"/{ns}/stop"),
            ],
            output="screen",
        )
    )

    # ── Static TFs for CMU local planner convention ──
    # sensor→vehicle: identity (LiDAR is approximately centered on Go2W)
    # sensor→camera: 90° rotation for camera convention (used by localPlanner visualization)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_vehicle_tf",
            arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
            remappings=tf_remaps,
            output="screen",
        )
    )
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_camera_tf",
            arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
            remappings=tf_remaps,
            output="screen",
        )
    )

    return nodes


def generate_launch_description():
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument(
                "robot_model", default_value="go2w",
                description=(
                    "Used by the nav2_mppi branch to pick the right per-platform "
                    "yaml: 'go2w' → nav2_go2w_full_stack.yaml (0.70 × 0.40 m "
                    "footprint, vx_max 0.50, base_link), 'go2' → "
                    "nav2_go2_full_stack.yaml (0.65 × 0.30 m, vx_max 0.30, "
                    "b_base_link). Other backends ignore this arg."
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_frame", default_value="world"),
            DeclareLaunchArgument("remap_tf", default_value="true"),
            DeclareLaunchArgument(
                "nav_backend",
                default_value="default",
                description="Local planner backend: default (default_nav.py — Python A* + "
                            "D* Lite + recovery), astar (astar_nav_node — C++ A* + "
                            "oriented footprint check), far (CMU autonomy stack). "
                            "Legacy aliases 'reactive' → 'default', 'rrt_star' → 'astar', "
                            "'far_rrt_star' → 'astar' (the reactive RRT* planner is gone).",
            ),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("map_topic", default_value=""),
            DeclareLaunchArgument(
                "registered_scan_topic",
                default_value="",
                description="3D PointCloud2 for FAR terrain analysis (must be in map frame for Cartographer mode)",
            ),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument(
                "nav_config",
                default_value=os.path.join(go2w_config_pkg, "config", "nav", "default_nav_single_go2w.yaml"),
            ),
            DeclareLaunchArgument(
                "cfpa2_config",
                default_value=os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml"),
            ),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="1.0"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.6"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="0.8"),
            DeclareLaunchArgument("cfpa2_min_utility", default_value="-0.5"),
            DeclareLaunchArgument("cfpa2_switch_hysteresis", default_value=""),
            DeclareLaunchArgument("cfpa2_goal_topic_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("max_linear_speed", default_value=""),
            DeclareLaunchArgument("require_settle_before_motion", default_value=""),
            DeclareLaunchArgument("nav_map_topic", default_value=""),
            DeclareLaunchArgument("far_max_speed", default_value="0.5"),
            DeclareLaunchArgument(
                "far_robot_id",
                default_value="0",
                description="Robot ID for FAR graph_msger (unique per robot in multi-robot setups)",
            ),
            # When false, skip the CFPA2 frontier picker — the caller supplies
            # goals from elsewhere (e.g. real CMU TARE). Default true for
            # back-compat with existing invocations.
            DeclareLaunchArgument("explore", default_value="true"),
            # Overrides for wiring FAR OUT of the exploration pipeline:
            # set far_goal_topic=""  → no publisher drives FAR's goal input;
            # set far_way_point_out to a dead sink so FAR's output doesn't
            # collide with a TARE-direct publication to /{ns}/way_point.
            DeclareLaunchArgument("far_goal_topic", default_value=""),
            DeclareLaunchArgument("far_way_point_out", default_value=""),
            # Default "Odometry" (relative) is correct for sim dual where each
            # Fast-LIO is wrapped in its own namespace. Real-robot launches set
            # this to "/Odometry" (absolute) because their Fast-LIO is started
            # un-namespaced. The adapter handles both forms (see fast_lio_tf_adapter.py).
            DeclareLaunchArgument("fast_lio_input_topic", default_value="Odometry"),
            # Real-robot launches override e.g. "nav2_go2_real.yaml" to swap
            # in a yaml retuned for the laptop's CPU + Mid-360 noise. Sim
            # leaves empty and gets nav2_go2{,w}_full_stack.yaml by robot_model.
            DeclareLaunchArgument("nav2_yaml_override", default_value=""),
            # Optional layered profile override on top of nav2_yaml_override,
            # used for small deltas like the holonomic Go2W real profile.
            DeclareLaunchArgument("nav2_yaml_extra_override", default_value=""),
            # Sim default: adapter publishes TF (sole base_link owner). Real
            # overrides to "false" because slam.launch.py already provides the
            # full TF chain map→camera_init→body→base_link plus map→odom static.
            DeclareLaunchArgument("fast_lio_publish_tf", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
