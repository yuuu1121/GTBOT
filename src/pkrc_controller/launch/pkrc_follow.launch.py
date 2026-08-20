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
                          'ned_convert': LaunchConfiguration('ned_convert'),
                          # 실시간 추종(2026-08-20): 1.0 m 잠금은 목표 갱신을 띄엄띄엄
                          # 만들어 플랫폼이 PKRC를 놓친다(사용자 관찰) — 0.4로 낮춤
                          'lock_threshold': 0.15}]),
        Node(package='pkrc_controller', executable='platform_follower',
             output='screen',
             # 실시간 추종: 플랫폼 플랜트는 지령 0.2에 실속도 0.06~0.11 m/s만 나온다
             # (leader_pilot 실측) — 지령 상한과 P이득을 올려 실속도를 끌어올린다
             # 편대 안전 대역(2026-08-20): 0.3 이상 연속 주행 시 팔로워 est가 0~18%로
             # 붕괴해 편대가 와해된다(실측) — 팔로워 추종 대역 내 저속 연속 추종.
             parameters=[{'v_max': 0.15, 'kp': 0.8}]),
    ])
