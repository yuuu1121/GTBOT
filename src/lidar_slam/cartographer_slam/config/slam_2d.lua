-- ============================================================================
-- Cartographer 2D SLAM configuration (Livox MID360)
-- ============================================================================
-- Parameter Types:
--   [Static]  - Cartographer does not support runtime parameter change
--              All values here are effectively static.
-- ============================================================================
-- Environment variables (set by launch file):
--   CARTOGRAPHER_INPUT_TYPE: "livox" or "pointcloud2"
--   CARTOGRAPHER_LOCALIZATION: "true" or "false"
-- ============================================================================
-- Copyright 2024
-- Livox Mid-360 2D SLAM unified configuration
--
-- Supports both Livox CustomMsg and PointCloud2 input types,
-- and both mapping and localization modes.
--
-- Usage:
--   ros2 launch cartographer_ros slam.launch.py
--   ros2 launch cartographer_ros slam.launch.py input_type:=pointcloud2
--   ros2 launch cartographer_ros slam.launch.py localization:=true load_state_filename:=/path/to/map.pbstream

include "map_builder.lua"
include "trajectory_builder.lua"

-- Read environment variables
local input_type = os.getenv("CARTOGRAPHER_INPUT_TYPE") or "livox"
local localization = os.getenv("CARTOGRAPHER_LOCALIZATION") == "true"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ==========================================================================
  -- FRAMES [Static]
  -- ==========================================================================
  -- TF tree: map -> odom -> base_link -> {imu_link, livox_link, sonar_link}
  -- Note: base_link children are published by boat_description URDF
  map_frame = "map",
  tracking_frame = "livox_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,         -- publish odom frame for consistency
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  -- ==========================================================================
  -- SENSORS [Static]
  -- ==========================================================================
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  num_livox_points = 0,

  -- ==========================================================================
  -- TF AND PUBLISHING [Static]
  -- ==========================================================================
  lookup_transform_timeout_sec = 0.2,  -- [s]
  submap_publish_period_sec = 0.3,     -- [s]
  pose_publish_period_sec = 5e-3,      -- [s]
  trajectory_publish_period_sec = 30e-3,  -- [s] NOTE: Set to 999999. to hide frozen trajectory in localization mode
  publish_odometry = true,             -- /slam/cartographer/odometry

  -- ==========================================================================
  -- SAMPLING RATIOS [Static]
  -- ==========================================================================
  -- 1.0 = use all data
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Input type configuration
if input_type == "livox" then
  options.num_livox_points = 1
else
  options.num_point_clouds = 1
end

-- ==========================================================================
-- MAP BUILDER [Static]
-- ==========================================================================
MAP_BUILDER.use_trajectory_builder_2d = true

-- ==========================================================================
-- TRAJECTORY BUILDER [Static]
-- ==========================================================================
-- Livox Mid-360 sensor characteristics
-- FOV: 360deg horizontal, -7 to +52 deg vertical
-- Range: 0.1m - 40m (70m @10% reflectivity)

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Range and height filtering for 3D to 2D projection
TRAJECTORY_BUILDER_2D.min_range = 0.3   -- [m]
TRAJECTORY_BUILDER_2D.max_range = 30.0  -- [m]
TRAJECTORY_BUILDER_2D.min_z = -0.5      -- [m]
TRAJECTORY_BUILDER_2D.max_z = 1.5       -- [m]

-- Voxel filter (downsampling)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- [m]

-- Adaptive voxel filter for submap insertion
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5   -- [m]
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.0   -- [m]

-- Loop closure scan matching
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9   -- [m]
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 30.0   -- [m]

-- Real-time correlative scan matcher (coarse matching)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- [m]
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- [rad] (20 deg)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher (fine matching)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- Motion filter (keyframe selection)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5     -- [s]
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2  -- [m]
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.)  -- [rad] (5 deg)

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- [m] grid cell size
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- ==========================================================================
-- POSE GRAPH [Static]
-- ==========================================================================
if localization then
  -- Pure localization mode
  TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
  }
  POSE_GRAPH.optimize_every_n_nodes = 20
else
  -- Mapping mode
  POSE_GRAPH.optimize_every_n_nodes = 35
end

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.max_num_final_iterations = 200

return options
