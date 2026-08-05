#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument('scenario', default_value='gtbot_world')
    rate_arg = DeclareLaunchArgument('simulation_rate', default_value='100.0')
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stonefish_ros2'), '/launch/simulator_gpu.launch.py']),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([FindPackageShare('gtbot_description'), '']),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('gtbot_description'), 'scenarios',
                [LaunchConfiguration('scenario'), '.scn']]),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'window_res_x': '1280',
            'window_res_y': '720',
            'rendering_quality': 'high',
        }.items())
    return LaunchDescription([scenario_arg, rate_arg, simulator])
