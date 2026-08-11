"""실물 gtbot(로봇 1대) 기동 — 시뮬과 동일한 제어 스택 + 하드웨어 어댑터.

구성(시뮬 대비 달라지는 것은 어댑터 계층뿐, 제어 로직은 동일):
  hwt9053_driver     RS485 AHRS -> /<robot>/imu (sensor_msgs/Imu)
  thruster_bridge    /<robot>/thrusters(4ch setpoint, 시뮬 규약) -> thruster_rpm(8ch RPM)
  thruster_can_node  thruster_rpm -> SocketCAN (can0, base_id 0x300)

사용: ros2 launch gtbot_formation hardware_robot.launch.py robot:=gtbot

피드백 구성(2026-08-10 확정, 플랫폼 중앙집중):
  헤딩 = 자체 hwt9053 AHRS(yaw_source='imu'), 위치·속도 = 플랫폼이 LiDAR로
  추정해 하행 전송하는 /<robot>/state_est(odom_source='est', 오차 4~5 cm 시뮬
  실측). 시뮬 게이트 회귀(2026-08-10): est 되먹임으로 S6(편대 성능 0.089 m) 통과,
  단 S4·S5 헤딩 게이트는 marginal 탈락(9.9~11.8° vs 10°) — est 잡음의 폐루프
  재유입로 판 헤딩 적합이 열화(sim-results.md 6차). 실기 캘리브레이션 시 참고.
  로봇에 위치 센서 불요. est·명령 두절 0.5 s 시 무추력 자연 정지(검증된
  안전 경로). 운용 요건: 초기 배치에서 반사판이 플랫폼 LiDAR에 보여야 est가
  성립한다(대략 지향이면 충분 — 강도 검출은 방향 관용).
멀티머신: 플랫폼·로봇 전부 같은 LAN + 동일 ROS_DOMAIN_ID(권장 42)면 DDS 기본
  멀티캐스트 디스커버리로 토픽이 이어진다. 오가는 것은 accel_cmd(하행)·
  state_est(하행)·(선택) imu(상행)뿐이라 WiFi 대역폭 부담 없음.
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
        # 이득은 시뮬 전 게이트 통과 구성(2026-08-10 프로브 H, 2/2 재현)과 동일 —
        # 실기 요 플랜트가 다르므로 kpsi는 출발점이며 물에서 캘리브레이션 항목.
        # k_yaw_off(자북↔플랫폼 프레임 정렬·IMU 드리프트 보정): 시뮬에서는 지속
        # 결합이 헤딩 지표를 열화시켜 0(비활성) — 실기는 프레임 정렬에 필수라 켠다.
        # 권장 절차: 배치 정지 구간에서 수렴 확인 후, 요동 결합 징후(자전·검출 명멸)
        # 가 보이면 0으로 내려 마지막 오프셋을 동결 운용.
        Node(package='gtbot_formation', executable='velocity_loop',
             parameters=[{'robot': robot, 'heading_mode': 'bearing',
                          'yaw_source': 'imu', 'odom_source': 'est',
                          'v_max': 0.2, 'kv': 1.0, 'kpsi': 0.18,
                          'e_deadband': 0.03, 'k_yaw_off': 0.005,
                          'vref_tau': 0.5}]),
    ])
