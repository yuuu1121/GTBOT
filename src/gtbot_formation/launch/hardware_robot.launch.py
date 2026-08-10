"""실물 gtbot(로봇 1대) 기동 — 시뮬과 동일한 제어 스택 + 하드웨어 어댑터.

구성(시뮬 대비 달라지는 것은 어댑터 계층뿐, 제어 로직은 동일):
  hwt9053_driver     RS485 AHRS -> /<robot>/imu (sensor_msgs/Imu)
  thruster_bridge    /<robot>/thrusters(4ch setpoint, 시뮬 규약) -> thruster_rpm(8ch RPM)
  thruster_can_node  thruster_rpm -> SocketCAN (can0, base_id 0x300)

사용: ros2 launch gtbot_formation hardware_robot.launch.py robot:=gtbot

잔여 통합 지점(실기 투입 전 필수):
  velocity_loop는 /<robot>/odometry(위치·속도·yaw)를 피드백으로 쓴다 — 시뮬 한정
  단순화 규약('온보드 센서 대역'). 실물에는 odometry 소스가 없으므로,
  (a) hwt9053 yaw + 속도 추정(GPS/DVL 등 추가 센서 결정 필요)으로 odometry 토픽을
  합성하는 노드를 붙이거나, (b) velocity_loop에 imu-yaw 전용 모드를 추가해야 한다.
  결정 전까지 velocity_loop는 이 launch에 넣지 않는다(무피드백 오동작 방지) —
  아래 주석 블록이 연결 예시다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='gtbot'),
        Node(package='hwt9053_driver', executable='hwt9053_node',
             namespace=robot,
             parameters=[{'frame_id': 'imu_link'}]),
        Node(package='thruster_control', executable='thruster_bridge',
             parameters=[{'robot': robot, 'max_rpm': 2000}]),
        Node(package='thruster_control', executable='thruster_can_node',
             parameters=[{'can_channel': 'can0', 'can_base_id': 0x300,
                          'num_thrusters': 8}]),
        # velocity_loop 연결 예시 (odometry 소스 확정 후 주석 해제):
        # Node(package='gtbot_formation', executable='velocity_loop',
        #      parameters=[{'robot': robot, 'heading_mode': 'bearing',
        #                   'v_max': 0.2}]),
    ])
