#!/usr/bin/env python3
# Copyright 2025
#
# BlueROV2 Simulation Launch File
#
# This launch file starts:
# - Stonefish simulator with BlueROV2 scenario
# - Thruster manager (optional)
#
# Usage:
#   # Scenario only
#   ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=false
#
#   # Scenario + Thruster Manager (default)
#   ros2 launch stonefish_ros2 bluerov2.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import math


def generate_launch_description():
    """Generate launch description for BlueROV2 simulation."""

    # Declare arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle namespace'
    )

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='bluerov2_infrustructure',
        description='Scenario name (without .scn extension)'
    )

    start_thruster_arg = DeclareLaunchArgument(
        'start_thruster_manager',
        default_value='true',
        description='Start thruster manager'
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='100.0',
        description='Simulation update rate (Hz)'
    )

    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='false',
        description='Use GPU rendering (set to true for GPU, false for headless)'
    )

    # Get configurations
    vehicle_name = LaunchConfiguration('vehicle_name')
    scenario = LaunchConfiguration('scenario')

    # 1. Stonefish Simulator
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stonefish_ros2'),
            '/launch/simulator_gpu.launch.py'
        ]),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([
                FindPackageShare('stonefish_description'), ''
            ]),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('stonefish_description'),
                'scenarios',
                [scenario, '.scn']
            ]),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'window_res_x': '2520',
            'window_res_y': '1440',
            'rendering_quality': 'high',
        }.items()
    )

    # 2. Thruster Manager (conditional)
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stonefish_thruster_manager'),
            '/launch/thruster_manager.launch.py'
        ]),
        launch_arguments={
            'vehicle_name': vehicle_name
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_thruster_manager'))
    )

    # Static TF: base_link -> base_link_frd (FRD coordinate frame)
    #
    # Problem: Stonefish publishes base_link with wrong orientation:
    #   Current: X=down, Y=back, Z=left
    #   Desired (FRD): X=forward, Y=right, Z=down
    #
    # Rotation matrix calculation:
    #   X_frd = -Y_base (forward = opposite of back)
    #   Y_frd = -Z_base (right = opposite of left)
    #   Z_frd = X_base (down = same as down)
    #
    # Rotation matrix: R = [[ 0, -1,  0],
    #                       [ 0,  0, -1],
    #                       [ 1,  0,  0]]
    #
    # Correct quaternion (verified via scipy):
    #   qx=0.5, qy=-0.5, qz=0.5, qw=-0.5
    #
    # Rotation matrix (column-wise interpretation):
    #   Col 0 (child X): [0, -1, 0] = -parent Y (forward) ✓
    #   Col 1 (child Y): [0, 0, -1] = -parent Z (right) ✓
    #   Col 2 (child Z): [1, 0, 0] = parent X (down) ✓

    base_link_frd_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_frd_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', [vehicle_name, '/base_link'],
            '--child-frame-id', [vehicle_name, '/base_link_frd']
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        scenario_arg,
        start_thruster_arg,
        simulation_rate_arg,
        simulator,
        thruster_manager,
        base_link_frd_publisher,
    ])
