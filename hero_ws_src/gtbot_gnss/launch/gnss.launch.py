import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('gtbot_gnss'),
        'config', 'gnss_params.yaml')

    # 데스크톱(DISPLAY)이 있으면 기본으로 지도 브라우저를 띄운다. SSH 등은 open_map:=false
    open_map_arg = DeclareLaunchArgument(
        'open_map', default_value='true' if os.environ.get('DISPLAY') else 'false',
        description='실시간 지도를 브라우저로 자동 오픈')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8000', description='실시간 지도 웹서버 포트')

    gnss_node = Node(
        package='gtbot_gnss',
        executable='gnss_node',
        name='gnss_node',
        parameters=[params, {'web_port': LaunchConfiguration('web_port')}],
        output='screen',
    )
    # 웹서버가 뜬 뒤 브라우저 오픈
    open_map = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['xdg-open', ['http://localhost:', LaunchConfiguration('web_port')]],
            output='log',
        )],
        condition=IfCondition(LaunchConfiguration('open_map')),
    )

    return LaunchDescription([open_map_arg, web_port_arg, gnss_node, open_map])
