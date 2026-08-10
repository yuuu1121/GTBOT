#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053 센서 노드 + RViz2 (ROS2)

센서 노드가 publish_tf:=true 로 base_link->imu_link TF 를 브로드캐스트하므로
RViz 의 imu_link 좌표축이 센서 회전을 따라 움직인다.
(별도 static TF 를 두면 같은 변환이 두 번 발행되어 충돌하므로 두지 않는다.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hwt9053_driver')
    rviz_config = os.path.join(pkg_share, 'rviz', 'imu_view.rviz')
    sensor_launch = os.path.join(pkg_share, 'launch', 'hwt9053.launch.py')

    # HWT9053 센서 노드 (TF 브로드캐스트 켜서 include)
    hwt9053 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch),
        launch_arguments={'publish_tf': 'true'}.items(),
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        hwt9053,
        rviz,
    ])
