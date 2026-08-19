#!/usr/bin/env python3
"""
Sensors Launch File
모든 센서를 한번에 실행

Usage:
    # UKFM만 실행
    ros2 launch pkrc_controller sensors.launch.py

    # UKFM + EKF 동시 실행
    ros2 launch pkrc_controller sensors.launch.py enable_ekf:=true

    # UKFM + EKF + 비교 로거 (3-way comparison)
    ros2 launch pkrc_controller sensors.launch.py enable_ekf:=true enable_comparison:=true
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Launch arguments
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_comparison = LaunchConfiguration('enable_comparison')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_ekf',
            default_value='false',
            description='Enable EKF localization (default: false, only UKFM)'
        ),
        DeclareLaunchArgument(
            'enable_comparison',
            default_value='false',
            description='Enable EKF vs UKFM comparison logger (requires enable_ekf:=true)'
        ),

        # # Microstrain IMU (GV7-INS)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('microstrain_inertial_driver'),
        #         '/launch/microstrain_launch.py'
        #     ]),
        # ),

        # # DVL-A50
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('dvl_a50'),
        #         '/launch/dvl_a50.launch.py'
        #     ]),
        # ),

        # ArUco Detector 6DOF (2Hz blinking marker optimized)
        # Disabled: not needed when using ros2 bag play (data already recorded)
        # Node(
        #     package='active_marker',
        #     executable='aruco_detector_6dof',
        #     name='aruco_detector_6dof',
        #     output='screen',
        #     parameters=[{
        #         'marker_ids': [0, 1, 2, 3],
        #         'marker_map_ids': [0, 1, 2, 3],
        #         'detection_hold_frames': 15,
        #         'exposure_time': 1,
        #         'brightness': -64,
        #         'contrast': 64,
        #         'gain': 0,
        #     }],
        # ),

        # # Pressure Sensor (MS5837)
        # Node(
        #     package='pressure_sensor',
        #     executable='pressure_sensor_node',
        #     name='pressure_sensor',
        #     output='screen',
        # ),

        # UKF-M Localization
        Node(
            package='pkrc_controller',
            executable='ukfm_localization',
            name='ukfm_localization',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'pressure_topic': '/pressure',
                'dvl_topic': '/dvl/data',
                'aruco_topic': '/aruco/pose_array',
                'use_dvl': True,
                'dvl_mount_yaw': 90.0,
            }],
        ),

        # EKF Localization (optional)
        Node(
            package='pkrc_controller',
            executable='ekf_localization_real',
            name='ekf_localization_real',
            output='screen',
            condition=IfCondition(enable_ekf),
            parameters=[{
                'frequency': 50.0,
                'imu_inverted': True,
                'imu_rotation_axis': 'x',
                'water_density': 1025.0,
                'atm_pressure': 101325.0,
                'gravity': 9.81,
                # 5-state model: [x, y, z, theta, v]
                'process_noise_Q': [0.03, 0.03, 0.001, 0.002, 0.02],
                'measurement_noise_R_full': [0.01, 0.01, 0.02, 0.01, 0.01],
                'measurement_noise_R_partial': [0.02, 0.01, 0.01],
            }],
        ),

        # 3-Way Comparison Logger: UKFM vs EKF vs 3-Sensor (requires enable_ekf:=true)
        Node(
            package='pkrc_controller',
            executable='comparison_data_logger',
            name='comparison_data_logger',
            output='screen',
            condition=IfCondition(enable_comparison),
            parameters=[{
                'output_dir': '/root/home/vlm_ws/src/plot_tools/csv_data',
                'log_rate': 10.0,
                'imu_inverted': True,
                'imu_rotation_axis': 'x',
                'dvl_transform_enabled': False,
            }],
        ),
    ])
