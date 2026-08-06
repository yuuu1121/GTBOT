from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    loops = [Node(package='gtbot_formation', executable='velocity_loop',
                  name=f'velocity_loop_{r}', parameters=[{'robot': r}])
             for r in ['gtbot', 'gtbot2', 'gtbot3']]
    return LaunchDescription(loops + [
        Node(package='gtbot_formation', executable='leader_pilot',
             parameters=[{'waypoints': [60.0, 0.0]}]),  # 직선 경로 — 게이트 S3 통과 구성(sim-results.md)
        Node(package='gtbot_formation', executable='koopman_formation'),
    ])
