import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_DIR = '/tmp/gtbot_formation_logs'   # 제어 진단 CSV(koopman 게이트·velocity_loop v_ref/ei)

def generate_launch_description():
    os.makedirs(LOG_DIR, exist_ok=True)
    # fallback_bearing: 스폰 배치(SPAWN_REL)에서 platform을 향하는 방위 — plate 마커는
    # 플랫폼 지향이어야 검출되므로, est 부재 시 폴백이 판을 플랫폼 쪽으로 유지한다.
    import math
    fb = {'gtbot': math.pi, 'gtbot2': -math.pi / 3, 'gtbot3': math.pi / 3}
    loops = [Node(package='gtbot_formation', executable='velocity_loop',
                  name=f'velocity_loop_{r}',
                  # v_max 0.5(기본) -> 0.2: LP는 항상 ±u_max 뱅뱅이라 v_ref가 v_max로 포화한
                  # 채 부호만 뒤집는 릴레이 한계사이클이 된다(S6 실측: v_ref 포화율 47~75%,
                  # a_cmd 부호 반전 ~2 Hz, 팔로워 평속 0.36 m/s = 리더 실속도 0.103 m/s의 3.5배,
                  # 목표대비 위치오차 평균 0.8~1.4 m). 리더가 0.103 m/s이므로 0.2면 2배 여유.
                  parameters=[{'robot': r, 'heading_mode': 'bearing', 'v_max': 0.2,
                               'fallback_bearing': fb[r],
                               'log_csv': f'{LOG_DIR}/vel_{r}.csv'}])
             for r in ['gtbot', 'gtbot2', 'gtbot3']]
    return LaunchDescription([
        # LiDAR 지각 범위(r<5 m) 제약: 워밍업 중 리더가 먼저 출발하면 gtbot이 지각 범위 밖으로
        # 밀려나 state_est가 영구 invalid → k가 안 늘어 S2 전환이 오지 않는다(S6 실측 확인).
        # start_leader:=false로 기동해 S2 전환(제어 진입) 확인 후 leader_pilot을 별도 실행한다.
        DeclareLaunchArgument('start_leader', default_value='true'),
    ] + loops + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            FindPackageShare('gtbot_formation'), '/launch/perception.launch.py'])),
        Node(package='gtbot_formation', executable='leader_pilot',
             parameters=[{'waypoints': [60.0, 0.0]}],  # 직선 경로 — 게이트 S3 통과 구성(sim-results.md)
             condition=IfCondition(LaunchConfiguration('start_leader'))),
        Node(package='gtbot_formation', executable='koopman_formation',
             # excite_div 8->16: plate 마커는 워밍업 가진의 로봇 이동이 검출을 깎아
             # 클린 틱 축적이 실속한다(실측: 359/600에서 최근 60s 클린율 3.6%).
             # 진폭 절반으로 이동을 줄여 검출 유지 — S2는 상대비교라 진폭에 강건 기대.
             # warmup_steps 600->450: plate 검출 가능 기하가 워밍업 중 소진돼 클린 틱이
             # ~506에서 실속(실측, 12분 관찰). S2는 bilinear<linear 상대비교라 표본 수에
             # 강건 기대 — 결과(S2 판정)로 검증한다.
             parameters=[{'state_source': 'lidar', 'excite_div': 16.0, 'warmup_steps': 200,
                          'log_csv': f'{LOG_DIR}/koopman.csv'}]),
    ])
