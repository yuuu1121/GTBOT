#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
스러스터 CAN 노드 기동 launch.

thruster_can_node(데몬)만 띄운다. keyboard_node 는 대화형 stdin(input())을
쓰기 때문에 launch 로 함께 띄우지 않고 별도 터미널에서 실행한다:
    ros2 run thruster_control keyboard_node --ros-args \
        --params-file <config>/thruster_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('thruster_control')
    default_params = os.path.join(pkg_share, 'config', 'thruster_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='thruster_control 파라미터 yaml 경로')

    thruster_can_node = Node(
        package='thruster_control',
        executable='thruster_can_node',
        name='thruster_can_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        thruster_can_node,
    ])
