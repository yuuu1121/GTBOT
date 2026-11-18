import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('serial_cmd'),
        'config', 'serial_cmd_params.yaml')

    serial_cmd_node = Node(
        package='serial_cmd',
        executable='serial_cmd_node',
        name='serial_cmd_node',
        parameters=[params],
        output='screen',
    )
    # input_node 는 /dev/tty 에서 직접 읽으므로 launch 터미널에 바로 입력 가능
    input_node = Node(
        package='serial_cmd',
        executable='input_node',
        name='serial_cmd_input',
        output='screen',
    )

    return LaunchDescription([
        serial_cmd_node,
        input_node,
        # q 로 입력을 끝내면 전체 종료
        RegisterEventHandler(OnProcessExit(
            target_action=input_node,
            on_exit=[EmitEvent(event=Shutdown(reason='input_node 종료'))],
        )),
    ])
