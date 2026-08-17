"""실물 platform(리더선) 기동 — OS0 실기 드라이버 + 시뮬과 동일한 지각·제어 스택.

구성:
  ouster-ros driver  실기 OS0 -> /ouster/points  (시뮬에서는 stonefish가 같은 토픽 공급)
  perception.launch  ouster_cluster(반사판 검출) + platform_perception(브리지)
                     — input_topic이 /ouster/points라 시뮬과 설정 동일
  koopman_formation  편대 제어 (state_source=lidar)

사용: ros2 launch gtbot_formation hardware_platform.launch.py sensor_hostname:=<OS0 IP>

플랫폼 IMU(2026-08-17 문서 정정): platform_perception은 /platform/imu(플랫폼 yaw)를
쓰고, 아래 hwt9053 노드가 namespace platform + imu/ddpm -> imu 리맵으로 그것을
공급한다 — **이미 구성돼 있다.** 종전 독스트링은 이 항목을 "잔여 통합 지점"으로
남겨두고 리맵 대상도 imu/data로 적었으나, 둘 다 틀렸다(드라이버가 내는 자세 토픽은
imu/ddpm이고 코드는 이미 그것을 리맵한다). 남은 것은 코드가 아니라 하드웨어 —
플랫폼에 hwt9053을 물리적으로 장착하고 RS485 포트를 확인하는 일이다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gtbot_formation')
    ouster_pkg = get_package_share_directory('ouster_ros')
    return LaunchDescription([
        # SHM 전용 DDS 프로파일 해제(2026-08-17 가드). 시뮬 런치는 512 KiB LiDAR
        # 메시지의 UDP 조각 유실을 피하려고 FASTRTPS_DEFAULT_PROFILES_FILE로
        # 공유메모리 전용 프로파일을 쓴다(useBuiltinTransports=false = UDP 끔).
        # **그 값이 셸에 export된 상태로 이 런치를 돌리면 기계 간 통신이 통째로
        # 끊긴다** — 실기는 플랫폼 1대 + 로봇 3대가 LAN으로 이어져야 한다.
        # 시뮬 실험 셸에서 그대로 실기로 넘어오는 사고를 막으려고 여기서 지운다.
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
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
             # 로봇과 같은 드라이버라 자세 토픽도 imu/ddpm — platform_perception은
             # 시뮬 scn과 같은 /platform/imu를 구독하므로 리맵해 그래프를 일치시킨다.
             # (ddpm 선택 근거는 hardware_robot.launch.py 주석 참조)
             remappings=[('imu/ddpm', 'imu')],
             parameters=[{'frame_id': 'imu_link'}]),
        Node(package='gtbot_formation', executable='koopman_formation',
             parameters=[{'state_source': 'lidar', 'excite_div': 16.0,
                          'warmup_steps': 200,
                          'require_odom': False}]),  # 실기: odometry 없음 — 가드·GT 진단 비활성
    ])
