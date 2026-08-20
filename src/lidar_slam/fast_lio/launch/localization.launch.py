"""
FAST-LIO Localization Launch File

Livox MID-360 LiDAR-Inertial Odometry for localization on a pre-built PCD map.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  map_path        : Path to the PCD map file (required)   (default: '')
  config_path     : Full path to FAST-LIO config yaml     (default: <pkg>/config/slam/mid360.yaml)
  use_sim_time    : Use simulation time                   (default: 'false')
  use_rviz        : Launch RViz                           (default: 'false')

================================================================================
TF TREE (provided by boat_description URDF)
================================================================================
  map
  └── odom
      └── base_link
          ├── livox_frame (LiDAR)
          ├── imu_link
          └── sonar_link

================================================================================
TOPICS
================================================================================
  Input:
    - /sensor/lidar/livox_mid360/points (livox_ros_driver2/CustomMsg)
    - /sensor/ins/livox_mid360/imu (sensor_msgs/Imu)
    - /slam/fast_lio/odometry (nav_msgs/Odometry) - from fast_lio
    - /slam/fast_lio/points_body (sensor_msgs/PointCloud2) - from fast_lio
  Output:
    - /slam/fast_lio/odometry (nav_msgs/Odometry)
    - /slam/fast_lio/points_body (sensor_msgs/PointCloud2)
    - /slam/fast_lio_loc/odometry (nav_msgs/Odometry) - map-aligned pose
    - /slam/fast_lio_loc/confidence (std_msgs/Float32)
    - /slam/fast_lio_loc/occupancy_grid (nav_msgs/OccupancyGrid, latched)
    - /slam/fast_lio_loc/map (sensor_msgs/PointCloud2, latched)

  Optional debug outputs (silent by default; opt in via config or CLI override):
    - /slam/fast_lio/debug/path           — `publish.path_en:=true`
    - /slam/fast_lio/debug/points_world   — `publish.scan_publish_en:=true`

================================================================================
EXAMPLES
================================================================================
  # Basic localization with pre-built map
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd

  # Localization without RViz
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd use_rviz:=false

  # Bag playback with simulation time
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd use_sim_time:=true

================================================================================
WORKFLOW
================================================================================
  1. First, create a map using mapping.launch.py
  2. Save the PCD map (automatically saved on shutdown or manually)
  3. Use this launch file with the saved map for localization
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    package_path = get_package_share_directory('fast_lio')
    boat_desc_path = get_package_share_directory('boat_description')

    default_config_path = os.path.join(package_path, 'config', 'slam', 'mid360.yaml')
    default_loc_config = os.path.join(package_path, 'config', 'localization', 'localization.yaml')
    default_rviz_config = os.path.join(package_path, 'rviz', 'localization.rviz')

    # Launch arguments
    map_path = LaunchConfiguration('map_path')
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the PCD map file (required)'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Full path to FAST-LIO config yaml file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RVIZ'
    )

    # FAST-LIO node (with map saving disabled for localization mode)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[
            config_path,
            {
                'use_sim_time': use_sim_time,
                'pcd_save.pcd_save_en': False,  # Disable map saving in localization mode
            }
        ],
        output='screen'
    )

    # Localization node
    localization_node = Node(
        package='fast_lio',
        executable='localization_node',
        name='localization_node',
        parameters=[
            default_loc_config,
            {
                'use_sim_time': use_sim_time,
                'map_path': map_path,
            }
        ],
        output='screen'
    )

    # Optional RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # Robot state publisher (boat_description URDF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boat_desc_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    # Declare arguments
    ld.add_action(declare_map_path_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    # Nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(fast_lio_node)
    ld.add_action(localization_node)
    ld.add_action(rviz_node)

    return ld
