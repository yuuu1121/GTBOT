#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053 원시데이터 + imu_filter_madgwick 융합 + RViz2 (ROS2)

센서 내장 자세(Euler/쿼터니언)는 pitch를 못 잡는 등 신뢰 불가.
대신 정상인 원시 데이터(/imu/data_raw = 가속도+자이로, /imu/mag = 자기장)를
imu_filter_madgwick 로 융합해 올바른 orientation(/imu/data)을 얻는다.
  - roll/pitch: 가속도+자이로 -> 정확
  - yaw: 자기장 -> 국소 간섭 심하면 use_mag:=false 로 (gyro 상대 yaw)

use_mag 인자로 자기장 사용 여부 토글:
  ros2 launch hwt9053_driver hwt9053_madgwick.launch.py use_mag:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hwt9053_driver')
    rviz_config = os.path.join(pkg_share, 'rviz', 'imu_view.rviz')
    sensor_launch = os.path.join(pkg_share, 'launch', 'hwt9053.launch.py')

    use_mag_arg = DeclareLaunchArgument(
        'use_mag', default_value='false',
        description='자기장으로 yaw 보정 여부. 이 로봇은 자기장 간섭이 심해(≈4255µT) '
                    '기본 false(자이로 상대 yaw, 시작 0). 절대 방위 필요시 true')

    # 센서 노드: 원시 데이터 공급용. 자체 orientation(imu/data)은 신뢰 불가하므로
    # madgwick 출력과 충돌 안 하게 imu/data_sensor 로 리맵. publish_tf 는 madgwick 담당.
    hwt9053 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch),
        launch_arguments={'publish_tf': 'false'}.items(),
    )

    # madgwick: 원시데이터 융합 -> orientation. 출력은 imu/data_filtered 로 리맵해
    # 센서 노드의 (신뢰불가) imu/data 와 충돌 방지. TF(base_link->imu_link)는 madgwick 담당.
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': ParameterValue(LaunchConfiguration('use_mag'), value_type=bool),
            'world_frame': 'enu',
            'publish_tf': True,
            'fixed_frame': 'base_link',   # base_link -> imu_link TF 발행
            'stateless': False,
            'remove_gravity_vector': False,
        }],
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/mag', 'imu/mag'),
            ('imu/data', 'imu/data_filtered'),  # 센서 노드 imu/data 와 충돌 방지
        ],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen',
    )

    return LaunchDescription([use_mag_arg, hwt9053, madgwick, rviz])
