"""플랫폼 쪽 추종 스택 — PKRC 월드좌표 계산 + 플랫폼 추종기.

시뮬: ros2 launch pkrc_controller pkrc_follow.launch.py
실기(플랫폼 PC, FAST-LIO 가동 전제):
  ros2 launch pkrc_controller pkrc_follow.launch.py \
      platform_odom_topic:=/Odometry ned_convert:=true
leader_pilot과 /platform/thrusters 를 다투므로 동시에 켜지 말 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform_odom_topic', default_value='/platform/odometry'),
        DeclareLaunchArgument('ned_convert', default_value='false'),
        Node(package='pkrc_controller', executable='pkrc_world_position',
             output='screen',
             parameters=[{'platform_odom_topic': LaunchConfiguration('platform_odom_topic'),
                          'ned_convert': LaunchConfiguration('ned_convert')}]),
        Node(package='pkrc_controller', executable='platform_follower',
             output='screen'),
    ])
