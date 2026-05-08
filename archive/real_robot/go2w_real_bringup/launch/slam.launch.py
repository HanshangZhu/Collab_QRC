#!/usr/bin/env python3
"""Real-robot SLAM launcher — selects between Cartographer+L1 and Fast-LIO+Mid360.

Contract (both backends must satisfy):
  - Publish TF: map → base_link (directly or via odom).
  - Provide a scan-compatible topic that pointcloud_to_laserscan can consume.

Cartographer (L1):
  - Subscribes to /utlidar/transformed_cloud (from transform_everything) + transformed_imu.
  - Publishes /<ns>/map_prob via cartographer_occupancy_grid_node.
  - Publishes TF map → body; a downstream static TF maps body → base_link.

Fast-LIO (Mid360):
  - Subscribes to /livox/lidar + /livox/imu.
  - Publishes TF camera_init → body (hardcoded frame names in laserMapping.cpp).
    This launch adds two static TFs so the nav stack sees map → base_link:
      map → camera_init     (identity rename)
      body → base_link      (identity — attach base_link as child of body)
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    slam = _get(context, "slam").strip().lower() or "carto_l1"
    mode_2d = _get(context, "carto_mode").strip().lower() == "2d"
    bringup_share = get_package_share_directory("go2w_real_bringup")

    actions = []

    if slam == "carto_l1":
        lua = "cartographer_l1_2d.lua" if mode_2d else "cartographer_l1_3d.lua"
        config_dir = os.path.join(bringup_share, "config", "slam")
        actions.append(
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                arguments=[
                    "-configuration_directory", config_dir,
                    "-configuration_basename", lua,
                ],
                remappings=[
                    ("points2", "/utlidar/transformed_cloud"),
                    # Cartographer 3D needs real gravity — use the raw (non-zeroed)
                    # transformed IMU. /utlidar/transformed_imu has orientation and
                    # linear_acceleration zeroed and will trigger an imu_tracker
                    # gravity-vector CHECK failure.
                    ("imu", "/utlidar/transformed_raw_imu"),
                ],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
                remappings=[("map", "/robot/map_prob")],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        )
        # Attach `body` beneath base_link so Cartographer's tracking_frame has
        # a TF path to the nav stack's base_link. Direction matters: Cartographer
        # publishes map → base_link (published_frame=base_link), so base_link
        # must remain root of the robot subtree. body hangs off as a child.
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_body_carto",
                arguments=[
                    "--frame-id", "base_link", "--child-frame-id", "body",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )
        # map → odom (identity) — Nav2's local_costmap is configured with
        # `global_frame: odom` (REP-105 convention), but Cartographer doesn't
        # publish an `odom` frame: it produces `map → base_link` directly,
        # then `base_link → body` is added above. controller_server's TF
        # lookup `odom → base_link` therefore fails with `Invalid frame ID
        # "odom"`. Same fix pattern as the fastlio_mid360 branch below
        # (added 2026-04-30): a static identity makes odom resolvable via
        # tree-walk through map. No multi-parenting risk — base_link stays
        # parented to `map` via Cartographer's dynamic; odom is a separate
        # child of map. The fast_lio_tf_adapter is not running in carto mode
        # so CLAUDE.md golden rule 16 ("adapter is the single owner of
        # odom→base_link TF") is preserved by scope: only relevant when
        # slam=fastlio_mid360. Independent of CHAMP's broken state_estimation
        # (golden rule "bypassed, not fixed" still applies — this fix
        # doesn't touch the EKF/leg-odom path).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity_carto",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "odom",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )

    elif slam == "fastlio_mid360":
        config_path = os.path.join(bringup_share, "config", "slam")
        mid360_json = os.path.join(config_path, "MID360_config.json")

        # ── livox_ros_driver2: reads UDP from Mid-360 @ 192.168.123.20,
        #    publishes /livox/lidar (Livox CustomMsg, xfer_format=1) + /livox/imu.
        #    Must start BEFORE fastlio_mapping so topics exist when it subscribes.
        #    frame_id="body" matches Fast-LIO's tracking frame (hardcoded in
        #    laserMapping.cpp: trans.child_frame_id="body"). The Mid-360's
        #    IMU↔LiDAR extrinsic is handled by fastlio_mid360.yaml extrinsic_T,
        #    so the ~4 cm offset doesn't need a separate TF frame.
        actions.append(
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                parameters=[
                    {"xfer_format": 1},                # Livox CustomMsg (what Fast-LIO expects)
                    {"multi_topic": 0},                # single /livox/lidar
                    {"data_src": 0},                   # 0=lidar, 1=lvx file
                    {"publish_freq": 10.0},
                    {"output_data_type": 0},
                    {"frame_id": "body"},              # match Fast-LIO's child frame
                    {"lvx_file_path": ""},
                    {"user_config_path": mid360_json},
                    {"cmdline_input_bd_code": "livox0000000001"},
                ],
                output="screen",
            )
        )

        actions.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                parameters=[
                    os.path.join(config_path, "fastlio_mid360.yaml"),
                    {"use_sim_time": False},
                ],
                output="screen",
            )
        )
        # Fast-LIO's actual TF output is hardcoded as `camera_init → body`
        # (see vendor/fast_lio/src/laserMapping.cpp: trans.child_frame_id = "body").
        # Note: earlier versions of this file added a body_lidar_to_base_link
        # static TF, which was a dangling dead-frame — `body_lidar` is never
        # part of Fast-LIO's chain. Removed 2026-04-17.
        #
        # Bridge Fast-LIO's naming to the nav stack's map→base_link expectation:
        #   map → camera_init    (identity, just renaming)
        #   (Fast-LIO dynamic)   camera_init → body
        #   body → base_link     (identity, attach base_link as child of body)
        # Gravity-align the map frame.
        #
        # Fast-LIO2 does NOT gravity-align its world frame at startup (see
        # vendor/fast_lio/src/IMU_Processing.hpp:195 — the line that would
        # rotate body init to make gravity = world-z is commented out).
        # Instead, Fast-LIO sets world = camera_init = body orientation
        # at t=0. If the Mid-360 is mounted tilted, camera_init is tilted
        # by the same amount in actual reality, and every downstream map
        # frame (/robot/map, /robot/octomap_*) inherits that tilt.
        #
        # Symptom: the whole structure in RViz looks tilted by the mount
        # angle even after the body→base_link calibration is correct.
        #
        # Fix: map → camera_init is no longer identity. It carries the
        # measured mount orientation so that RViz, with fixed_frame=map,
        # sees voxels in a gravity-aligned frame.
        #
        # Values match the measurement from tools/measure_mid360_tilt.py
        # (2026-04-17): body is tilted (roll=-2.109°, pitch=+15.103°)
        # relative to gravity, so camera_init (= body at t=0) has the
        # SAME orientation, and the TF map→camera_init expresses camera_init
        # in map frame — i.e., the positive version of the measured tilt.
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_camera_init",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "camera_init",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll",  "-0.036809",
                    "--pitch",  "0.263591",
                    "--yaw",    "0",
                ],
                output="log",
            )
        )
        # Mid-360 mount calibration (measured 2026-04-17 via
        # tools/measure_mid360_tilt.py, 20 s, std=0.027 m/s²):
        #   IMU body reads gravity as (-0.260, -0.036, +0.964) g
        #   → body is tilted +15.103° pitch, -2.109° roll relative to
        #     level base_link.
        #   Static TF that puts base_link back to level therefore
        #   applies the INVERSE rotation: roll=+0.036809, pitch=-0.263591.
        #   Yaw is unobservable from gravity; assume 0 unless measured
        #   separately. Translation is unobservable from IMU; measure
        #   with a tape measure and fill in if the Mid-360 is offset
        #   from base_link (typical Go2 top-plate mount has body ~8 cm
        #   forward and ~12 cm above base_link; left at 0 here because
        #   the raycast/nav error at 5 cm octomap resolution is small).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link_fastlio",
                arguments=[
                    "--frame-id", "body", "--child-frame-id", "base_link",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll",  "0.036809",
                    "--pitch", "-0.263591",
                    "--yaw",   "0",
                ],
                output="log",
            )
        )
        # map → odom (identity) — phantom frame so Nav2's local_costmap can
        # resolve TF(odom → base_link) via tree walk through the existing
        # chain (odom → map (inv) → camera_init → body → base_link). On real
        # there is no loop-closure correction yet, so map and odom are
        # functionally identical; once SC-PGO ports cleanly, this static
        # gets replaced by SC-PGO's dynamic map→odom + the adapter takes
        # over odom→base_link as the canonical REP-105 split.
        # The fast_lio_tf_adapter on real is launched with publish_tf=false
        # (see real_single.launch.py) precisely to avoid double-parenting
        # base_link (body→base_link static above is already its parent).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "odom",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )

    else:
        raise ValueError(f"Unknown slam backend '{slam}' (expected carto_l1 or fastlio_mid360)")

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("slam", default_value="carto_l1"),
            DeclareLaunchArgument("carto_mode", default_value="3d", description="2d or 3d"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
