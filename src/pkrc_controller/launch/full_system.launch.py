#!/usr/bin/env python3
"""
PKRC Full System Launch File
모든 센서 + 컨트롤러를 한번에 실행

Usage:
    ros2 launch pkrc_controller full_system.launch.py
    ros2 launch pkrc_controller full_system.launch.py target_distance:=1.5 target_depth:=2.0

Note: DVL-A50과 압력센서 노드는 별도 패키지에서 실행됩니다.
    - DVL-A50: ros2 run dvl_a50 dvl_a50 또는 dvl-a50 패키지 clone 후 빌드
    - 압력센서: ros2 run pressure_sensor pressure_sensor_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pkrc_controller')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.0',
            description='Target distance from wall (m)'
        ),
        DeclareLaunchArgument(
            'target_depth',
            default_value='1.0',
            description='Target depth (m)'
        ),
        DeclareLaunchArgument(
            'dvl_address',
            default_value='192.168.1.99',
            description='DVL-A50 IP address'
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can0',
            description='CAN bus channel'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='wall_following',
            description='Controller mode: wall_following or depth_only'
        ),

        # Pressure sensor node (MS5837)
        Node(
            package='pressure_sensor',
            executable='pressure_sensor_node',
            name='pressure_sensor',
            output='screen',
        ),

        # DVL-A50 node
        Node(
            package='dvl_a50',
            executable='dvl_a50_sensor',
            name='dvl_a50_node',
            output='screen',
            parameters=[{
                'dvl_ip_address': LaunchConfiguration('dvl_address'),
            }],
        ),

        # Wall following controller (includes depth control)
        Node(
            package='pkrc_controller',
            executable='wall_following',
            name='wall_following_controller',
            output='screen',
            parameters=[{
                'target_distance': LaunchConfiguration('target_distance'),
                'target_depth': LaunchConfiguration('target_depth'),
                'can_channel': LaunchConfiguration('can_channel'),
                'enabled': True,
                'max_current': 5.0,
                'current_scale': 3.0,
                # Wall following PID
                'kp': 1.5,
                'ki': 0.1,
                'kd': 0.2,
                'approach_speed': 0.4,
                'follow_speed': 0.3,
                # Depth PID
                'depth_kp': 0.8,
                'depth_ki': 0.35,
                'depth_kd': 1.2,
            }],
        ),
    ])
