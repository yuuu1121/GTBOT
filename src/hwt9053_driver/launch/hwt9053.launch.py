#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053-485 AHRS IMU 센서 드라이버 launch (ROS2)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hwt9053_driver')
    default_params = os.path.join(pkg_share, 'config', 'hwt9053_params.yaml')

    # 런치 인자
    port_arg = DeclareLaunchArgument('port', default_value='/dev/hwt9053')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='9600')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='imu_link')
    publish_rate_arg = DeclareLaunchArgument('publish_rate', default_value='50')
    publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='false')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    # 센서 노드
    # 파라미터는 yaml 파일 먼저 로드하고, 런치 인자로 개별 오버라이드한다.
    hwt9053_node = Node(
        package='hwt9053_driver',
        executable='hwt9053_node',
        name='hwt9053_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=int),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        publish_rate_arg,
        publish_tf_arg,
        params_file_arg,
        hwt9053_node,
    ])
