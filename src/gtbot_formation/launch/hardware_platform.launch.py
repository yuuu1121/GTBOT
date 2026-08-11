"""실물 platform(리더선) 기동 — OS0 실기 드라이버 + 시뮬과 동일한 지각·제어 스택.

구성:
  ouster-ros driver  실기 OS0 -> /ouster/points  (시뮬에서는 stonefish가 같은 토픽 공급)
  perception.launch  ouster_cluster(반사판 검출) + platform_perception(브리지)
                     — input_topic이 /ouster/points라 시뮬과 설정 동일
  koopman_formation  편대 제어 (state_source=lidar)

사용: ros2 launch gtbot_formation hardware_platform.launch.py sensor_hostname:=<OS0 IP>

잔여 통합 지점: platform_perception은 /platform/imu(플랫폼 yaw)를 쓴다 — 실물
플랫폼에 hwt9053을 달아 namespace platform으로 올리고 imu/data -> imu 리맵을
걸면 충족된다(드라이버 기본 토픽이 imu/data라 리맵 없이는 안 물린다).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gtbot_formation')
    ouster_pkg = get_package_share_directory('ouster_ros')
    return LaunchDescription([
        DeclareLaunchArgument('sensor_hostname', default_value='os-sensor.local'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ouster_pkg, 'launch', 'driver.launch.py')),
            launch_arguments={
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'viz': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'perception.launch.py'))),
        Node(package='hwt9053_driver', executable='hwt9053_node',
             namespace='platform',
             # 드라이버 기본 토픽은 imu/data — platform_perception은 시뮬 scn과 같은
             # /platform/imu를 구독하므로 리맵해 시뮬·실기 그래프를 일치시킨다.
             remappings=[('imu/data', 'imu')],
             parameters=[{'frame_id': 'imu_link'}]),
        Node(package='gtbot_formation', executable='koopman_formation',
             parameters=[{'state_source': 'lidar', 'excite_div': 16.0,
                          'warmup_steps': 200,
                          'require_odom': False}]),  # 실기: odometry 없음 — 가드·GT 진단 비활성
    ])
