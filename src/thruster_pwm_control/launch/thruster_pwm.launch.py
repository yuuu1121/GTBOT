import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('thruster_pwm_control'),
        'config', 'thruster_pwm_params.yaml')

    pwm_node = Node(
        package='thruster_pwm_control',
        executable='pwm_node',
        name='pwm_node',
        parameters=[params],
        output='screen',
    )
    # keyboard_node 는 /dev/tty 에서 직접 키를 읽으므로 launch 터미널에서 바로 조작 가능
    keyboard_node = Node(
        package='thruster_pwm_control',
        executable='keyboard_node',
        name='keyboard_node',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        pwm_node,
        keyboard_node,
        # z/ESC 로 텔레옵을 끝내면 전체 종료 (pwm_node 는 종료 시 중립 출력)
        RegisterEventHandler(OnProcessExit(
            target_action=keyboard_node,
            on_exit=[EmitEvent(event=Shutdown(reason='keyboard_node 종료'))],
        )),
    ])
