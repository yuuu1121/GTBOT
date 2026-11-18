#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
헤딩 유지 캐스케이드 PID 전체 스택.

  IMU(hwt9053) + madgwick ─▶ cascade_controller ◀─ heading_teleop(cmd_vel)
                                   │ thruster_pwm_cmd
                                   ▼
                               pwm_node ─▶ PCA9685 ─▶ ESC

인자:
  use_pwm:=false   pwm_node 없이 드라이런 (thruster_pwm_cmd 만 확인, 모터 안 돎)
  use_teleop:=false  키보드 텔레옵 없이 (외부에서 cmd_vel 발행)
  use_mag:=true    madgwick 에 자기장 사용 (기본 false: 자이로 상대 yaw)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('thruster_pwm_control')
    pwm_params = os.path.join(share, 'config', 'thruster_pwm_params.yaml')
    hold_params = os.path.join(share, 'config', 'heading_hold_params.yaml')
    imu_launch = os.path.join(get_package_share_directory('hwt9053_driver'),
                              'launch', 'hwt9053_madgwick.launch.py')

    use_pwm = DeclareLaunchArgument('use_pwm', default_value='true')
    use_teleop = DeclareLaunchArgument('use_teleop', default_value='true')
    use_mag = DeclareLaunchArgument('use_mag', default_value='false')

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch),
        launch_arguments={
            'use_rviz': 'false',
            'use_mag': LaunchConfiguration('use_mag'),
        }.items(),
    )

    controller = Node(
        package='thruster_pwm_control', executable='cascade_controller_node',
        name='cascade_controller', parameters=[hold_params], output='screen',
    )

    pwm = Node(
        package='thruster_pwm_control', executable='pwm_node', name='pwm_node',
        parameters=[pwm_params], output='screen',
        condition=IfCondition(LaunchConfiguration('use_pwm')),
    )

    teleop = Node(
        package='thruster_pwm_control', executable='heading_teleop_node',
        name='heading_teleop', parameters=[hold_params], output='screen',
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    return LaunchDescription([
        use_pwm, use_teleop, use_mag,
        imu, controller, pwm, teleop,
        RegisterEventHandler(OnProcessExit(
            target_action=teleop,
            on_exit=[EmitEvent(event=Shutdown(reason='heading_teleop 종료'))],
        )),
    ])
