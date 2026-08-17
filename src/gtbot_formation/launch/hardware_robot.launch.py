"""실물 gtbot(로봇 1대) 기동 — 시뮬과 동일한 제어 스택 + 하드웨어 어댑터.

구성(시뮬 대비 달라지는 것은 어댑터 계층뿐, 제어 로직은 동일):
  hwt9053_driver     RS485 AHRS -> imu/ddpm, 리맵으로 /<robot>/imu (sensor_msgs/Imu)
  thruster_bridge    /<robot>/thrusters(4ch setpoint, 시뮬 규약) -> thruster_rpm(8ch RPM)
  thruster_can_node  thruster_rpm -> SocketCAN (can0, base_id 0x300)

사용: ros2 launch gtbot_formation hardware_robot.launch.py robot:=gtbot

피드백 구성(2026-08-10 확정, 플랫폼 중앙집중):
  헤딩 = 자체 hwt9053 AHRS(yaw_source='imu') — **실측으로 재검토 필요**:
  정지 55분 로그(imu_run1)에서 imu/ddpm yaw가 32°/h로 흘렀고 Allan 편차가 전
  구간 +1 기울기라 절대 기준(자기계)이 없다. 순간 잡음은 0.0013°로 우수하나
  10분 주행에 5°가 밀린다. 이 구성으로는 장시간 편대가 성립하지 않으므로,
  LiDAR 헤딩 추정을 절대 기준으로 삼고 IMU를 단기 자이로로 쓰는 융합
  (k_yaw_off 경로)이 실기 기본이 되어야 한다. 상세 research/sim-results.md.
  위치·속도 = 플랫폼이 LiDAR로
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
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    return LaunchDescription([
        # SHM 전용 DDS 프로파일 해제(2026-08-17 가드). 시뮬 런치는 512 KiB LiDAR
        # 메시지의 UDP 조각 유실을 피하려고 FASTRTPS_DEFAULT_PROFILES_FILE로
        # 공유메모리 전용 프로파일을 쓴다(useBuiltinTransports=false = UDP 끔).
        # **그 값이 셸에 export된 상태로 이 런치를 돌리면 기계 간 통신이 통째로
        # 끊긴다** — 실기는 플랫폼 1대 + 로봇 3대가 LAN으로 이어져야 한다.
        # 시뮬 실험 셸에서 그대로 실기로 넘어오는 사고를 막으려고 여기서 지운다.
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
        DeclareLaunchArgument('robot', default_value='gtbot'),
        Node(package='hwt9053_driver', executable='hwt9053_node',
             namespace=robot,
             # 실기 드라이버가 내는 자세 토픽은 imu/ddpm — velocity_loop는 시뮬 scn과
             # 같은 /<robot>/imu를 구독하므로 리맵해 시뮬·실기 그래프를 일치시킨다.
             # ddpm을 고른 근거(imu_run1, 정지 55분): 같은 로그의 세 출력 중 yaw
             # 드리프트가 raw 1024°/h, gp 256°/h, ddpm 32°/h로 ddpm이 최선이고,
             # imu/raw는 orientation_covariance=-1(자세 미제공)이라 애초에 못 쓴다.
             remappings=[('imu/ddpm', 'imu')],
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
