#!/usr/bin/env python3
# Copyright 2025
#
# BlueBoat Simulation Launch File
#
# This launch file starts:
# - Stonefish simulator with BlueBoat scenario
# - Thruster manager (optional)
#
# Usage:
#   # Scenario only
#   ros2 launch stonefish_ros2 blueboat.launch.py start_thruster_manager:=false
#
#   # Scenario + Thruster Manager (default)
#   ros2 launch stonefish_ros2 blueboat.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for BlueBoat simulation."""

    # Declare arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='blueboat',
        description='Vehicle namespace'
    )

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='blueboat_sea',
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
            'window_res_x': '960',
            'window_res_y': '1056',
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

    return LaunchDescription([
        vehicle_name_arg,
        scenario_arg,
        start_thruster_arg,
        simulation_rate_arg,
        simulator,
        thruster_manager,
    ])
