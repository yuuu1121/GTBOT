#!/usr/bin/env python3
"""
PKRC Wall Following Launch File
DVL-A50 + MS5837 압력센서 + VESC 스러스터로 벽 따라가기

Usage:
    ros2 launch pkrc_controller wall_following.launch.py
    ros2 launch pkrc_controller wall_following.launch.py target_distance:=1.5 target_depth:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.0',
            description='Target distance from wall (m)'
        ),
        DeclareLaunchArgument(
            'target_depth',
            default_value='0.0',
            description='Target depth (m)'
        ),
        DeclareLaunchArgument(
            'approach_speed',
            default_value='0.4',
            description='Approach speed (0-1)'
        ),
        DeclareLaunchArgument(
            'follow_speed',
            default_value='0.3',
            description='Follow speed along wall (0-1)'
        ),
        DeclareLaunchArgument(
            'max_current',
            default_value='5.0',
            description='Maximum current per thruster (A)'
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can0',
            description='CAN bus channel'
        ),

        # Wall following controller node
        Node(
            package='pkrc_controller',
            executable='wall_following',
            name='wall_following_controller',
            output='screen',
            parameters=[{
                'target_distance': LaunchConfiguration('target_distance'),
                'target_depth': LaunchConfiguration('target_depth'),
                'approach_speed': LaunchConfiguration('approach_speed'),
                'follow_speed': LaunchConfiguration('follow_speed'),
                'max_current': LaunchConfiguration('max_current'),
                'can_channel': LaunchConfiguration('can_channel'),
                'enabled': True,
                # Wall following PID
                'kp': 1.5,
                'ki': 0.1,
                'kd': 0.2,
                'yaw_gain': 0.1,
                'approach_threshold': 0.3,
                'surge_trim': 0.95,
                'sway_trim': 1.05,
                # Depth PID
                'depth_kp': 0.8,
                'depth_ki': 0.35,
                'depth_kd': 1.2,
                'depth_offset': 0.0,
                'current_scale': 3.0,
            }],
            remappings=[
                # DVL-A50 driver publishes to /dvl/data
                # Pressure sensor publishes to /pressure
            ],
        ),
    ])
