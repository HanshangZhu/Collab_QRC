"""3D-frontier-exploration launch wrapper.

Composes the canonical nav_test_mujoco_fastlio.launch.py with:
  - demo_ramp.xml scene (override mujoco_model_path)
  - spawn at (2, 0) — west end of ramp scene
  - nvblox_frontend mapper_node (CUDA 3D mapping)

For CFPA2 ig_dimension=3d, edit
src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml
(or pass a yaml overlay via --ros-args -p) — the base launch hardcodes the
config path. The cfpa2_single_robot_3d.yaml overlay sitting next to it
contains the 3-line diff (planning_map_topic_suffix + ig_dimension +
voxels_3d_topic_suffix) ready to copy in.

A future tidy-up is to wire `cfpa2_config_path` as a LaunchArgument in the
base launch; deferred so this wrapper stays purely additive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Resolve the workspace root to point at the demo_ramp scene by default.
    # (This launch file lives in <ws>/src/go2w/go2_gazebo_sim/launch/.)
    here = os.path.dirname(os.path.realpath(__file__))
    ws_root = os.path.realpath(os.path.join(here, "..", "..", "..", ".."))
    default_scene = os.path.join(
        ws_root, "src", "go2w", "go2_gazebo_sim", "mujoco", "demo_ramp.xml")

    # Config paths for the new trav pipeline (Phase 4–5).
    elevation_cupy_share = get_package_share_directory("elevation_mapping_cupy")
    trav_share = get_package_share_directory("trav_cost_filters")
    cfpa2_share = get_package_share_directory("cfpa2_collaborative_autonomy")
    emap_core_params = os.path.join(
        elevation_cupy_share, "config", "core", "core_param.yaml")
    emap_setup_params = os.path.join(
        trav_share, "config", "elevation_mapping.yaml")
    default_trav_weights = os.path.join(
        elevation_cupy_share, "config", "core", "weights.dat")
    filter_chain_params = os.path.join(
        trav_share, "config", "grid_map_filters.yaml")
    cfpa2_demo_ramp_overlay = os.path.join(
        cfpa2_share, "config", "cfpa2_single_robot_demo_ramp.yaml")

    args = [
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="2.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "cfpa2_executable_suffix",
            default_value="",
            description="Suffix for cfpa2_single_robot_node ('_cpp' = pure C++).",
        ),
        DeclareLaunchArgument(
            "explore", default_value="true",
            description="true: CFPA2 autonomous frontier exploration. "
                        "false: CFPA2 disabled; drive manually by clicking "
                        "RViz 'Nav2 Goal' (publishes /robot/goal_pose)."),
        DeclareLaunchArgument(
            "robot_seed_radius_m",
            default_value="1.4",
            description="UNKNOWN-ONLY forced-free disk radius (m) seeded around "
                        "the robot in /robot/traversability_grid every frame. "
                        "Fills ONLY unknown cells (unseen ground); scanned cells "
                        "(walls) are left untouched (see stamp_free_disk "
                        "unknown_only). Sized to the MEASURED ground blind cone: "
                        "with the -15deg Mid-360 tilt applied, flat ground first "
                        "returns at ~1.25 m (empty return hole is only ~0.43 m; "
                        "near walls return from ~0.43 m). 1.4 m = blind cone + "
                        "margin so the spawn free-bubble connects to the sensed "
                        "ground ring (BFS allow_unknown=false can cross) WITHOUT "
                        "falsely clearing genuinely-unexplored unknown beyond the "
                        "robot. Was 3.5 m (sized for the UNTILTED -7deg blind "
                        "cone ~3.25 m, which ignored the tilt and erased near "
                        "walls). Measured 2026-05-23."),
        DeclareLaunchArgument(
            "upper_bound_clearance", default_value="true",
            description="Enable Miki et al. 2022 Sec. II-H upper_bound overhang "
                        "rescue in grid_map_to_occupancy_grid. When the "
                        "ray-cast upper_bound is much lower than the cell's "
                        "elevation, the elevation point came from an overhang "
                        "above clear floor (bridge, ceiling) — cell is forced "
                        "free. Useful for indoor scenes with overhanging "
                        "structures (e.g. slam_ops2). Off by default."),
        # nvblox_frontend knobs
        DeclareLaunchArgument("nvblox_voxel_size_m", default_value="0.10"),
        DeclareLaunchArgument(
            "enable_nvblox_mapper", default_value="false",
            description="Enable the optional nvblox_frontend mapper for "
                        "/<ns>/voxels_3d. Default false because the ETH "
                        "elevation_mapping_cupy traversability path does not "
                        "need nvblox and many dev machines do not have the "
                        "vendored nvblox CUDA library built."),
        # Costmap source: '3d' swaps both global+local StaticLayers to read
        # /robot/traversability_grid so planner and MPPI treat ramps as free.
        # Pass nav_costmap_mode:=2d to revert to the octomap-based baseline.
        DeclareLaunchArgument("nav_costmap_mode", default_value="3d",
            description="'3d': both costmaps use traversability_grid. "
                        "'2d': default octomap /robot/map (baseline)."),
        DeclareLaunchArgument(
            "enable_legacy_2d_proj", default_value="false",
            description="Re-enable mapper_node's legacy 2D traversability "
                        "projection. Default false; the planned "
                        "elevation_mapping_cupy + grid_map filter pipeline "
                        "(docs/claude/plans/2026-05-14-trav-grid-rewrite.md) "
                        "owns /<ns>/traversability_grid when this is false. "
                        "Set true for A/B comparison or fallback."),
        DeclareLaunchArgument(
            "has_wheels", default_value="true",
            description="True spawns Go2W (wheel-legged hybrid + "
                        "go2w_hybrid_cmd_router). False spawns pure Go2 "
                        "(12-DoF leg only, CHAMP cmd_vel_legged direct, no "
                        "router). Forwarded to nav_test_mujoco_fastlio."),
        DeclareLaunchArgument(
            "trav_weight_file", default_value=default_trav_weights,
            description="Path to the traversability CNN weights.dat pickle. "
                        "Default = baseline ETH weights shipped in "
                        "elevation_mapping_cupy/config/core. Point at a "
                        "fine-tuned file (e.g. training_runs/weights_*.dat) "
                        "to A/B test without touching the baseline."),
        DeclareLaunchArgument(
            "cfpa2_config_overlay", default_value=cfpa2_demo_ramp_overlay,
            description="Absolute path to a CFPA2 param overlay YAML applied "
                        "on top of cfpa2_single_robot.yaml. Default = the "
                        "demo_ramp overlay (16 m ramp box). For the ops2 "
                        "building corridor pass cfpa2_single_robot_ops2.yaml "
                        "(allow_unknown=false, unlimited goal distance) — see "
                        "scripts/launch/nav_test_slam_ops2_v4_go2.sh."),
    ]

    # Reuse the full fastlio launch — it handles MuJoCo, Point-LIO/Fast-LIO,
    # CFPA2, Nav2, RViz, and all the supporting plumbing.
    base_launch_path = os.path.join(
        get_package_share_directory("go2_gazebo_sim"),
        "launch",
        "nav_test_mujoco_fastlio.launch.py")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
            "spawn_x":   LaunchConfiguration("spawn_x"),
            "spawn_y":   LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "rviz": LaunchConfiguration("rviz"),
            "gui":  LaunchConfiguration("gui"),
            "explore": LaunchConfiguration("explore"),
            "cfpa2_executable_suffix": LaunchConfiguration("cfpa2_executable_suffix"),
            "nav_costmap_mode": LaunchConfiguration("nav_costmap_mode"),
            "has_wheels": LaunchConfiguration("has_wheels"),
            # A verified ramp viewpoint means the Go2W should climb with
            # controlled wheel drive instead of treating the segment as open
            # flat-ground cruise. The trigger remains sensor-derived; no
            # scene-coordinate corridor is configured.
            "ramp_force_legged_enabled": "false",
            "ramp_force_wheel_enabled": "true",
            "ramp_goal_mode_topic": "ramp_ascent_goal_mode",
            "ramp_goal_stale_sec": "3.0",
            "ramp_force_max_vx_mps": "0.17",
            "ramp_force_max_yaw_rate_rps": "0.20",
            # CFPA2 param overlay (default demo_ramp; ops2 wrapper passes
            # cfpa2_single_robot_ops2.yaml). demo_ramp tightens frontier
            # filters for 2 m ramp corridors; ops2 disables allow_unknown
            # (no leak through unknown behind hand-walls) + unlimited goal
            # distance for the 70 m corridor. See each yaml's header.
            "cfpa2_config_overlay": LaunchConfiguration("cfpa2_config_overlay"),
        }.items(),
    )

    # nvblox_frontend mapper node — delayed 5 s so any residual preflight
    # activity from a previous session finishes before the mapper starts.
    # respawn=True guards against SIGTERM from a concurrent preflight.
    mapper = Node(
        package="nvblox_frontend",
        executable="mapper_node",
        name="nvblox_frontend_mapper",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(LaunchConfiguration("enable_nvblox_mapper")),
        parameters=[{
            # cloud_topic and odom_topic intentionally not set here.
            # The node's namespace (robot) prefixes relative defaults
            # cloud_registered_body → /robot/cloud_registered_body and
            # odom/nav → /robot/odom/nav, matching Fast-LIO + slam_odom_relay.
            "use_sim_time":      LaunchConfiguration("use_sim_time"),
            "world_frame":       "map",
            "voxel_size_m":      LaunchConfiguration("nvblox_voxel_size_m"),
            "publish_period_s":  0.5,
            # 40m × 40m world-fixed grid: covers demo_ramp (24×16m) plus
            # margin, so historical observations persist as the robot moves.
            "trav_xy_extent_m":  40.0,
            "voxel_xy_extent_m": 20.0,
            "voxel_z_extent_m":  3.0,
            "voxel_z_origin_m":  -0.5,
            "slope_max_deg":     30.0,
            "step_max_m":        0.20,
            "robot_clearance_m": 0.50,
            "enable_legacy_2d_proj": LaunchConfiguration("enable_legacy_2d_proj"),
        }],
    )

    # NOTE: frontier_3d_test_node removed — it visualised nvblox VoxelGrid3D
    # clusters from the optional mapper. With the ETH elevation + ramp_safe
    # fusion producing a clean 2D /traversability_grid, CFPA2 runs in 2D mode
    # (planning_map_topic_suffix=/traversability_grid, ig_dimension=2d, see
    # cfpa2_single_robot.yaml) so the 3D voxel cluster viz is dead weight.
    # Re-add the node alongside enable_nvblox_mapper=true if you want the
    # 3D voxel-cluster IG path back.

    # ---- Traversability pipeline (nav_costmap_mode:=3d only) ---------------
    # elevation_mapping_cupy → filter_chain_runner → grid_map_to_occupancy_grid
    # All three are delayed 6 s (1 s after the nvblox mapper) so SLAM + MuJoCo
    # are ready and no concurrent preflight kills the mapper before it starts.
    is_3d = IfCondition(
        PythonExpression(["'", LaunchConfiguration("nav_costmap_mode"), "' == '3d'"])
    )

    # 1. elevation_mapping_cupy: Kalman-fused height map on GPU.
    #    Publishes /<node_name>/elevation_map_raw = /elevation_mapping/elevation_map_raw
    #    → remapped to /<ns>/elevation_map_raw so the filter chain picks it up.
    #    TF is read from /<ns>/tf + /<ns>/tf_static (namespaced per CLAUDE.md rule 4).
    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[emap_core_params, emap_setup_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time"),
                     "weight_file": LaunchConfiguration("trav_weight_file")}],
        remappings=[
            # elevation_mapping_cupy hardcodes topic as f"/{self.get_name()}/{pub_key}".
            # With name="elevation_mapping" that is /elevation_mapping/elevation_map_raw.
            ("/elevation_mapping/elevation_map_raw",
             ["/", LaunchConfiguration("robot_namespace"), "/elevation_map_raw"]),
            # Namespace the TF streams (CLAUDE.md golden rule #4 + #10).
            ("/tf",        ["/", LaunchConfiguration("robot_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"]),
        ],
        condition=is_3d,
    )

    # 2. filter_chain_runner: 10-stage grid_map_filters chain.
    #    elevation_map_raw (3 layers) → elevation_map_filtered (12 layers).
    filter_runner = Node(
        package="trav_cost_filters",
        executable="filter_chain_runner",
        name="filter_chain_runner",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[filter_chain_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=is_3d,
    )

    # 3. grid_map_to_occupancy_grid: traversability layer → OccupancyGrid.
    #    Output /<ns>/traversability_grid consumed by Nav2 StaticLayer costmap.
    occ_adapter = Node(
        package="trav_cost_filters",
        executable="grid_map_to_occupancy_grid",
        name="grid_map_to_occupancy_grid",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "input_topic":     "elevation_map_filtered",
            "output_topic":    "traversability_grid",
            # CNN ↔ analytical fusion: trav_fused = max(CNN_traversability,
            # ramp_safe). CNN catches walls the analytical chain misses;
            # ramp_safe rescues clear ramp-body slopes the CNN over-rejects,
            # while shallow ramp-foot transitions fall back to CNN/mid-band
            # cost. See grid_map_filters.yaml for the trapezoidal envelope.
            "traversability_layer": "trav_fused",
            # Conservative thresholds: only cells with trav ≥ 0.60 are
            # treated as cost-0 free; cells in [0.30, 0.60] get the costly
            # mid-band 1-99 interpolation so MPPI/planner steers AROUND
            # them when an obviously-free route exists; trav < 0.30 is
            # outright lethal. This stops the planner from grazing
            # ramp-foot / wall-edge cells whose local geometry is ambiguous.
            "free_threshold":  0.60,
            # Lowered 0.30 → 0.05 (aggressive). The current trav_fused has
            # excess noisy low values from CNN+ramp_safe combined; even 0.08
            # still produced too many lethal cells choking the costmap. At
            # 0.05, only cells with very high CNN-confidence in "non-
            # traversable" become hard-lethal; the rest stays in the
            # mid-band where Nav2 inflation does footprint-aware spread.
            "lethal_threshold": 0.05,
            # Height-based extra cost — discourages planning over elevated
            # surfaces (ramp, platform) when a flat-ground route reaches
            # the same frontier. Cells at h=0.05m → 0 cost; h=1.00m → 90
            # cost (just below the 100 lethal). Combined with trav-cost
            # via max() so lethal walls stay lethal.
            "elevation_cost_enabled": False,
            "elevation_layer":        "elevation",
            "elevation_cost_min_h":   0.05,
            "elevation_cost_max_h":   1.50,
            "elevation_cost_max_value": 90,
            # Dynamic stability margin around platform/cliff edges. This uses
            # the measured step_height layer and a robot-scale proximity
            # radius; it is not a scene-coordinate keep-out zone.
            # Disabled 2026-05-17: this layer applied a circular Gaussian-like
            # cost halo around any step_height ≥ 0.30m cell, which double-bumps
            # against Nav2's own InflationLayer (radius 0.60m, footprint-aware).
            # The double inflation produced visible "bleed" of high cost into
            # otherwise free cells along walls/cliffs. Nav2's inflation alone
            # is now the single source of truth for footprint-aware spread;
            # cliff cells still propagate via the trav layer hitting the
            # lethal_threshold, just without the extra circular halo.
            "cliff_proximity_cost_enabled": False,
            "cliff_step_layer": "step_height",
            "cliff_proximity_radius_m": 0.25,
            "cliff_step_threshold_m": 0.30,
            "cliff_step_saturation_m": 0.45,
            "cliff_proximity_cost_max_value": 90,
            "upper_bound_clearance_enabled": LaunchConfiguration("upper_bound_clearance"),
            "upper_bound_layer": "upper_bound",
            "upper_bound_overhang_threshold_m": 0.30,
            "upper_bound_clear_cost": 0,
            "seed_robot_footprint": True,
            "robot_frame": "base_link",
            # Manual-goal mode (explore:=false) needs a larger initial cleared
            # disk so the planner can route the very first user-clicked goal
            # out of the spawn pose before elevation_mapping has filled in the
            # surroundings. CFPA2 autonomous mode keeps the tight 0.65 m seed.
            "robot_seed_radius_m": LaunchConfiguration("robot_seed_radius_m"),
            "seed_max_clear_cost": 50,
            "ramp_override_enabled": True,
            "slope_layer": "slope",
            "step_residual_layer": "step_residual",
            "ramp_min_slope_rad": 0.20943951023931956,
            "ramp_max_slope_rad": 0.5235987755982988,
            "ramp_max_step_residual_m": 0.06,
            # elevation_mapping_cupy is a robot-centered rolling map. Project
            # each frame into a fixed world grid before Nav2/RViz consumes it;
            # otherwise unknown holes and one-frame obstacle hits make the
            # traversability display change shape continuously.
            "fixed_grid_enabled": True,
            # 100×100 m world-fixed grid at 0.10 m/cell, centred at the origin
            # (spawn). Covers slam_ops2 (80×32 m) AND the full ±35 m corridor
            # exploration target with margin (x,y ∈ [-50,+50]). demo_ramp
            # (16×16) fits too.
            # 2026-05-20: shrunk from 200×200 m (2000² = 4 MB/msg). The huge
            # grid (a) rendered black in RViz near GL_MAX_TEXTURE_SIZE, (b)
            # caused cross-host DDS backpressure, and (c) made SmacHybrid's
            # allow_unknown search wander a 4 M-cell costmap → ~17 s planner
            # timeouts (the NO_PLAN stuck loop). 1000² = 1 MB/msg, ~4× cheaper.
            "fixed_origin_x": -50.0,
            "fixed_origin_y": -50.0,
            "fixed_width_cells": 1000,
            "fixed_height_cells": 1000,
            "unknown_clears_history": False,
            # Preserve high-but-traversable costs (e.g. elevation/cliff
            # stability cost 90). Only true OccupancyGrid lethal cells are
            # temporally confirmed into persistent obstacles.
            "occupied_cost_threshold": 100,
            "free_cost_threshold": 30,
            "occupied_confirm_hits": 2,
            "occupied_clear_hits": 0,
            "max_hit_count": 8,
            # Walls must come from sensor data (elevation_mapping + slope/step
            # filters), not from a scene-specific hardcoded rectangle. The
            # workspace_mask_* knobs remain in the node for ad-hoc scene-bound
            # debugging but are OFF by default for exploration correctness.
            "workspace_mask_enabled": False,
        }],
        remappings=[
            ("/tf",        ["/", LaunchConfiguration("robot_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"]),
        ],
        condition=is_3d,
    )

    # 4. (REMOVED) Slope-verified ramp viewpoint goals.
    #
    # ramp_ascent_goal_node used to publish /<ns>/ramp_ascent_goal here,
    # producing PointStamped viewpoints whenever the filtered GridMap layers
    # showed a traversable ramp patch. This was removed because:
    #   - On flat indoor scenes (slam_ops2) the slope/step layers fire on
    #     stair-step mesh artifacts → CFPA2 chases phantom ramp goals.
    #   - Even when disabled via `cfpa2_single_robot.yaml: ramp_ascent_enabled:
    #     false`, the node kept publishing the topic and stayed alive across
    #     restarts (no kill pattern matched), so the next launch saw two ramp
    #     goal publishers racing.
    #   - On demo_ramp the rest of the autonomy stack (Nav2 SmacPlannerLattice
    #     + MPPI on the trav_grid) can drive the robot onto the ramp once the
    #     planner sees the ramp cells as traversable; the explicit viewpoint
    #     goal was an optimisation, not a hard requirement.
    # If ramp viewpoint goals are needed again, restore this Node and add
    # `ramp_ascent_goal_node` to scripts/launch/_preflight_kill.sh.

    # Static identity: base_link → body
    # Fast-LIO hardcodes cloud_registered_body.header.frame_id = "body"
    # (laserMapping.cpp:564). elevation_mapping_cupy looks up map→body to
    # transform each cloud into the map frame. Our TF tree only has
    # map→odom→base_link; "body" is absent. fast_lio_tf_adapter already
    # treats body ≡ base_link (it republishes the odom→body pose as
    # odom→base_link) but never publishes the explicit link.
    # Adding base_link→body identity closes map→odom→base_link→body so
    # safe_lookup_transform(map, body) succeeds and terrain data is integrated.
    body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_body_tf",
        namespace=LaunchConfiguration("robot_namespace"),
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "body",
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
        ],
        remappings=[("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"])],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=is_3d,
    )

    # Delay mapper + frontier viz by 5 s. The preflight kill script targets
    # "mapper_node" by name; if a second launch attempt runs ≤5 s into the
    # first, the mapper would be killed before MuJoCo even starts. A 5 s
    # delay means the mapper starts after MuJoCo+SLAM are up and any
    # concurrent preflight has already finished. respawn=True above gives a
    # second layer of protection if it's still killed.
    deferred = TimerAction(period=5.0, actions=[mapper])
    # Trav pipeline nodes start 1 s after the nvblox mapper.
    deferred_trav = TimerAction(
        period=6.0,
        actions=[elevation_mapping, filter_runner, occ_adapter],
    )

    return LaunchDescription([*args, base_launch, body_tf, deferred, deferred_trav])
