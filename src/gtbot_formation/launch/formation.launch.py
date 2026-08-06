from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    loops = [Node(package='gtbot_formation', executable='velocity_loop',
                  name=f'velocity_loop_{r}', parameters=[{'robot': r}])
             for r in ['gtbot', 'gtbot2', 'gtbot3']]
    return LaunchDescription(loops + [
        Node(package='gtbot_formation', executable='leader_pilot'),
        Node(package='gtbot_formation', executable='koopman_formation'),
    ])
