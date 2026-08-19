#!/usr/bin/env python3
"""
PKRC Depth Controller Launch File
MS5837 압력센서 + VESC heave 스러스터로 깊이 유지

Usage:
    ros2 launch pkrc_controller depth_controller.launch.py
    ros2 launch pkrc_controller depth_controller.launch.py target_depth:=2.0 standalone_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'target_depth',
            default_value='1.0',
            description='Target depth (m)'
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
        DeclareLaunchArgument(
            'standalone_mode',
            default_value='false',
            description='If true, only controls heave. If false, also handles teleop.'
        ),

        # Depth controller node
        Node(
            package='pkrc_controller',
            executable='depth_controller',
            name='depth_controller',
            output='screen',
            parameters=[{
                'target_depth': LaunchConfiguration('target_depth'),
                'max_current': LaunchConfiguration('max_current'),
                'can_channel': LaunchConfiguration('can_channel'),
                'standalone_mode': LaunchConfiguration('standalone_mode'),
                'enabled': True,
                # Depth PID
                'kp': 0.8,
                'ki': 0.35,
                'kd': 1.2,
                'depth_offset': 0.0,
                'current_scale': 3.0,
            }],
        ),
    ])
