"""명령 PC 용: broadcast_node 만 실행. 터미널 입력이 모든 Pi 에 예약 발행된다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('serial_cmd'),
        'config', 'serial_cmd_params.yaml')

    broadcast_node = Node(
        package='serial_cmd',
        executable='broadcast_node',
        name='serial_cmd_broadcast',
        parameters=[params, {'lead_time': LaunchConfiguration('lead_time')}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('lead_time', default_value='1.0',
                              description='입력 -> 실행까지 여유 시간(초)'),
        broadcast_node,
        RegisterEventHandler(OnProcessExit(
            target_action=broadcast_node,
            on_exit=[EmitEvent(event=Shutdown(reason='broadcast_node 종료'))],
        )),
    ])
