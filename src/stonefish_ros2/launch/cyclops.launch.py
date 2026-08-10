#!/usr/bin/env python3
# Cyclops Simulation Launch File
#
# Usage:
#   ros2 launch stonefish_ros2 cyclops.launch.py
#
# Cyclops (SF 30k class) imported from uuv_cyclops_description into Stonefish.
# Note: start_thruster_manager defaults to false — the thruster manager needs a
# cyclops-specific allocation config; enable it once that config exists.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Cyclops simulation."""

    # Declare arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='cyclops',
        description='Vehicle namespace'
    )

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='cyclops_world',
        description='Scenario name (without .scn extension)'
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='100.0',
        description='Simulation update rate (Hz)'
    )

    start_thruster_arg = DeclareLaunchArgument(
        'start_thruster_manager',
        default_value='false',
        description='Start thruster manager (needs a cyclops allocation config)'
    )

    # Get configurations
    vehicle_name = LaunchConfiguration('vehicle_name')
    scenario = LaunchConfiguration('scenario')

    # Stonefish Simulator
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
            'window_res_x': '1920',
            'window_res_y': '1080',
            'rendering_quality': 'high',
        }.items()
    )

    # Thruster Manager
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

    # Static TF: base_link -> base_link_enu
    base_link_frd_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_frd_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '1', '--qy', '0', '--qz', '0', '--qw', '0',
            '--frame-id', [vehicle_name, '/base_link'],
            '--child-frame-id', [vehicle_name, '/base_link_enu']
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        scenario_arg,
        simulation_rate_arg,
        start_thruster_arg,
        simulator,
        thruster_manager,
        base_link_frd_publisher,
    ])
