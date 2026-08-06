from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    loops = [Node(package='gtbot_formation', executable='velocity_loop',
                  name=f'velocity_loop_{r}', parameters=[{'robot': r, 'heading_mode': 'bearing'}])
             for r in ['gtbot', 'gtbot2', 'gtbot3']]
    return LaunchDescription([
        # LiDAR 지각 범위(r<5 m) 제약: 워밍업 중 리더가 먼저 출발하면 gtbot이 지각 범위 밖으로
        # 밀려나 state_est가 영구 invalid → k가 안 늘어 S2 전환이 오지 않는다(S6 실측 확인).
        # start_leader:=false로 기동해 S2 전환(제어 진입) 확인 후 leader_pilot을 별도 실행한다.
        DeclareLaunchArgument('start_leader', default_value='true'),
    ] + loops + [
        Node(package='gtbot_formation', executable='platform_perception'),
        Node(package='gtbot_formation', executable='leader_pilot',
             parameters=[{'waypoints': [60.0, 0.0]}],  # 직선 경로 — 게이트 S3 통과 구성(sim-results.md)
             condition=IfCondition(LaunchConfiguration('start_leader'))),
        Node(package='gtbot_formation', executable='koopman_formation',
             parameters=[{'state_source': 'lidar'}]),
    ])
