"""수신 Pi 용: serial_cmd_node 만 실행 (입력 없음, 부팅 자동 실행용)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('serial_cmd'),
        'config', 'serial_cmd_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='',
                              description='이 Pi 식별자 (비우면 hostname)'),
        Node(
            package='serial_cmd',
            executable='serial_cmd_node',
            name='serial_cmd_node',
            parameters=[params, {'robot_id': LaunchConfiguration('robot_id')}],
            output='screen',
        ),
    ])
