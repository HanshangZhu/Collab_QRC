-- Cartographer 3D config for Unitree Go2 UTLidar
-- Based on TiNredmc/sim_carto_l1_ros2 tuning for Unitree 4D L1 LiDAR
-- Adapted for real-time Go2 ethernet operation

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "body",               -- transform_everything publishes IMU in body frame
  published_frame = "base_link",         -- see cartographer_l1_2d.lua for rationale
  odom_frame = "odom",
  provide_odom_frame = false,            -- avoid tf_bridge deadlock (sim lesson, 2026-04-06)
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                 -- single point cloud from UTLidar
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,       -- 200Hz pose output
  publish_tracked_pose = true,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.use_trajectory_builder_3d = true

-- Local SLAM
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  -- UTLidar: one PointCloud2 = one revolution
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 21.0

-- Disable online correlative matcher — too expensive for 3D, kills processing rate
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- Use constant-velocity extrapolator (NOT imu_based)
-- The imu_based one integrates accelerometer → 14km+ drift due to gravity compensation errors
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001

-- Ceres scan matcher (weights = initial-guess constraint, LOW = trust scan match more)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5e-1  -- low: let scan match determine position
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5e0     -- moderate: gyro is okay
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

-- Motion filter — avoid inserting submaps on tiny movements
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(5)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.0

-- Global SLAM (loop closure)
POSE_GRAPH.optimize_every_n_nodes = 60  -- match num_range_data per TiNredmc guide
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.constraint_builder.min_score = 0.45  -- was 0.55 but found 0 matches; gentle weights protect us
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 3.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
-- Gentle optimization weights — corrections are smooth, not violent (TiNredmc: 1e-2 range)
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-2
POSE_GRAPH.optimization_problem.rotation_weight = 1e-2
POSE_GRAPH.global_constraint_search_after_n_seconds = 3

-- Logging
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = true  -- see what's matching
POSE_GRAPH.optimization_problem.log_solver_summary = false

return options
