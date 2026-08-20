"""
Launch file for robot_state_publisher with boat URDF.

This launch file publishes the static TF transforms for the boat platform:
- base_link -> livox_frame (LiDAR)
- base_link -> imu_link (IMU)
- base_link -> sonar_link (Sonar sensors)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true.'
    )

    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('boat_description'),
        'urdf',
        'boat.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
    ])
