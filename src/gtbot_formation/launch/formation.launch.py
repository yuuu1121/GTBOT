import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LOG_DIR = '/tmp/gtbot_formation_logs'   # 제어 진단 CSV(koopman 게이트·velocity_loop v_ref/ei)

def generate_launch_description():
    os.makedirs(LOG_DIR, exist_ok=True)
    # fallback_bearing: 스폰 배치(SPAWN_REL)에서 platform을 향하는 방위 — plate 마커는
    # 플랫폼 지향이어야 검출되므로, est 부재 시 폴백이 판을 플랫폼 쪽으로 유지한다.
    # OFFSETS에서 유도한다(하드코딩 금지) — 편대 회전 노브
    # GTBOT_FORMATION_ROT_DEG를 쓰면 스테이션 방위가 바뀌므로 폴백도 따라가야 한다.
    import math
    from gtbot_formation.relative_state import OFFSETS
    fb = {r: math.atan2(-o[1], -o[0])
          for r, o in zip(['gtbot', 'gtbot2', 'gtbot3'], OFFSETS)}
    loops = [Node(package='gtbot_formation', executable='velocity_loop',
                  name=f'velocity_loop_{r}',
                  # v_max 0.5(기본) -> 0.2: LP는 항상 ±u_max 뱅뱅이라 v_ref가 v_max로 포화한
                  # 채 부호만 뒤집는 릴레이 한계사이클이 된다(S6 실측: v_ref 포화율 47~75%,
                  # a_cmd 부호 반전 ~2 Hz, 팔로워 평속 0.36 m/s = 리더 실속도 0.103 m/s의 3.5배,
                  # 목표대비 위치오차 평균 0.8~1.4 m). 리더가 0.103 m/s이므로 0.2면 2배 여유.
                  parameters=[{'robot': r, 'heading_mode': 'bearing', 'v_max': 0.2,
                               'yaw_source': 'imu',  # 로봇 자체 AHRS 헤딩(실기 hwt9053 동일 의미론)
                               # 실기 동일: 위치·속도 = 플랫폼 LiDAR est(완전 출력 피드백, 사용자 결정
                               # 2026-08-10 "시뮬도 똑같이 추정값"). est 잡음의 폐루프 재유입로 S4·S5
                               # 헤딩이 9.9~11.8°로 열화(odom 대비 ~2배) — S6(임무 성능)은 통과.
                               'odom_source': 'est',
                               # k_yaw_off(2026-08-12 재활성): 프로브 A에서 OFF로 둔 이유는
                               # 시뮬 IMU에 드리프트가 없어(1e-4°) 보정할 대상이 없는데 결합만
                               # 남아 S4를 열화시켰기 때문이다. 실기 실측(imu_run1)에서 IMU가
                               # 32°/h로 흐르는 것이 확인돼 전제가 뒤집혔다 — 이제 보정할 대상이
                               # 있다. 주행 중 준정지 게이트(|gyro_z|<0.05) 개방률 68~82% 실측,
                               # est 10 Hz -> EMA 시상수 약 27 s, 32°/h 램프의 정상상태 지연
                               # 0.24°, 판 헤딩 잡음 0.5°는 400 표본 평균으로 0.025°까지 감쇠.
                               'k_yaw_off': 0.005,
                               'e_deadband': 0.03,  # 프로브 B 판정: 한계 개선(최악 13.3→11.8°) — 유지
                               # 공전의 마지막 고리: 순수 적분 v_ref가 실측 속도와 100~107°
                               # 어긋나 감속 지령이 회전으로 소비됐다(실측). 0.5 s로 되끈다.
                               'vref_tau': 0.5,
                               'kv': 1.0,  # 프로브 C: 단독 무효였으나 기생토크 감소 방향 — 유지
                               # kpsi 0.15(F): S5 8~12.5→6~7° 첫 통과. 재현 런에서 gtbot3만
                               # 9.5~10.2° 경계 플립(sim 상수 외력의 로봇별 요 기생토크 비대칭)
                               # → 0.18(H)로 여유 확보. ζ는 낮아지나 캡 0.06 미포화 영역 유지.
                               'kpsi': 0.18,

                               # est_hold_s(2026-08-17): 검출 신선도 창. 기본 0.5는 종전
                               # 동작이며, LiDAR 발행률이 열화해 검출 간격이 이를 넘으면
                               # 추력이 통째로 끊긴다(붕괴 런 실측: 틱의 83~87%). 캠페인에서
                               # 인자로 늘려 A/B 하려고 노출한다.
                               'est_hold_s': ParameterValue(
                                   LaunchConfiguration('est_hold_s'), value_type=float),
                               'fallback_bearing': fb[r],
                               'log_csv': f'{LOG_DIR}/vel_{r}.csv'}])
             for r in ['gtbot', 'gtbot2', 'gtbot3']]
    # 실기 동일 액추에이터 경로(2026-08-10): 제어 출력(4ch setpoint)을 하드웨어
    # 인터페이스(thruster_rpm, 8ch RPM)로 변환해 흘리고, 시뮬 끝단에서만 되돌린다
    # — 실기에서는 rpm_to_sim 대신 thruster_can_node가 같은 토픽을 소비한다.
    bridges = [Node(package='thruster_control', executable='thruster_bridge',
                    name=f'thruster_bridge_{r}', parameters=[{'robot': r}])
               for r in ['gtbot', 'gtbot2', 'gtbot3']]
    rpm_sims = [Node(package='gtbot_formation', executable='rpm_to_sim',
                     name=f'rpm_to_sim_{r}', parameters=[{'robot': r}])
                for r in ['gtbot', 'gtbot2', 'gtbot3']]
    # IMU 드리프트 심(2026-08-12): scn은 /<robot>/imu_true를 내고 심이 실측 드리프트를
    # 얹어 /<robot>/imu로 재발행한다. Stonefish의 yaw_drift가 결정론적 램프뿐이라
    # 실측의 배회를 못 내는 것을 메우는 자리 — 상세는 imu_drift_shim 독스트링.
    # seed를 로봇마다 다르게 줘 세 대가 같은 궤적으로 흐르지 않게 한다(공통모드로
    # 흐르면 상대 기하가 보존돼 문제가 실제보다 순해진다).
    drift_shims = [Node(package='gtbot_formation', executable='imu_drift_shim',
                        name=f'imu_drift_shim_{r}',
                        parameters=[{'robot': r, 'seed': ParameterValue(
                            PythonExpression([LaunchConfiguration('seed_base'), '+', str(i)]),
                            value_type=int)}])
                   for i, r in enumerate(['gtbot', 'gtbot2', 'gtbot3', 'platform'])]
    # platform 포함(2026-08-13): 플랫폼 IMU가 로봇과 같은 기종으로 확정돼 같은 드리프트를
    # 받는다. 플랫폼 yaw 오차는 platform_perception의 yaw_p를 통해 세 로봇의 월드 헤딩
    # 추정에 1:1로 전파되므로, 빼놓으면 저하 시나리오가 실제보다 순해진다.
    return LaunchDescription([
        # Fast DDS 공유메모리 프로파일(2026-08-17). /ouster/points 한 스캔이 512 KiB인데
        # Fast DDS 기본 SHM 세그먼트가 정확히 512 KiB라 이 메시지는 SHM에 못 들어가고
        # UDP로 폴백한다 — 1.5 KB MTU로 ~350 조각이 되어 하나만 잃어도 샘플 전체가
        # 버려진다. 발행은 10.01 Hz인데 구독은 3~5 Hz(C++ 노드는 0.77 Hz)였던 원인이다.
        # 세그먼트를 8 MiB로 키우면 구독이 9.75~10.02 Hz로 실물과 같아진다(실측).
        # 이 컨테이너는 net.core.rmem_max가 sysctl 미노출이라 커널 버퍼 조정은 불가.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', PathJoinSubstitution([
            FindPackageShare('gtbot_description'), 'config', 'fastdds_shm.xml'])),
        # BLAS 스레드 고정(2026-08-17 계측): numpy가 코어 수(32)만큼 OpenBLAS 워커를
        # 띄우는데, 이 노드들의 행렬은 분할 이득이 없을 만큼 작아 동기화 비용만 남는다
        # — RLS 갱신 실측이 32스레드 16.30 ms vs 1스레드 11.63 ms로 **많이 쓸수록 느리다**.
        # 게다가 OpenBLAS는 병렬 구간 사이에 워커를 재우지 않고 스핀시켜, 실계산이
        # 1코어의 23%인데 ps에는 1866%(18코어)로 잡힌다. (**정정**: 이 헛돎이 LiDAR
        # 발행률을 떨어뜨린다고 적었으나 실측으로 부정됐다 — CPU를 1.6%로 내려도
        # 발행률은 그대로였다. 진짜 원인은 DDS 전송 유실이었고 위 SHM 프로파일이
        # 그것을 고친다. 이 설정은 죽은 계산·헛돎 제거로서 여전히 유효하다.)
        # 첫 액션이어야 이후 노드·포함 런치가 모두 상속한다(numpy import 전에 걸려야 함).
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        # LiDAR 지각 범위(r<5 m) 제약: 워밍업 중 리더가 먼저 출발하면 gtbot이 지각 범위 밖으로
        # 밀려나 state_est가 영구 invalid → k가 안 늘어 S2 전환이 오지 않는다(S6 실측 확인).
        # start_leader:=false로 기동해 S2 전환(제어 진입) 확인 후 leader_pilot을 별도 실행한다.
        DeclareLaunchArgument('start_leader', default_value='true'),
        # koopman_formation 제어 팔: 'mpc'(기본, 불변성 보강 사전 Θ + H스텝 MPC) | 'analytic'(참 그래디언트) |
        # 'model'(원논문 꼴 bilinear Θ 1-step). mpc 노브는 koopman_node 파라미터 기본값 참조.
        DeclareLaunchArgument('controller', default_value='mpc'),
        DeclareLaunchArgument('mpc_rho', default_value='3.0'), DeclareLaunchArgument('mpc_sat', default_value='1.5'),
        DeclareLaunchArgument('mpc_horizon', default_value='2'), DeclareLaunchArgument('mpc_fit_seed', default_value='0'),
        DeclareLaunchArgument('mpc_lam', default_value='0.0'), DeclareLaunchArgument('mpc_sub', default_value='1'),
        DeclareLaunchArgument('lidar_yaw_bias', default_value='0.0'),
        DeclareLaunchArgument('det_dropout', default_value='0.0'),
        DeclareLaunchArgument('dropout_seed', default_value='0'),
        DeclareLaunchArgument('dropout_mode', default_value='uniform'),
        DeclareLaunchArgument('inc_phi_zero', default_value='50.0'),
        DeclareLaunchArgument('phi_log', default_value=''),
        DeclareLaunchArgument('seed_base', default_value='100'),
        DeclareLaunchArgument('est_hold_s', default_value='0.5'),
    ] + loops + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            FindPackageShare('gtbot_formation'), '/launch/perception.launch.py']),
            # 저하 시나리오 주입을 지각 스택으로 전달(기본 0 = 종전 동작)
            launch_arguments={'lidar_yaw_bias': LaunchConfiguration('lidar_yaw_bias'),
                              'det_dropout': LaunchConfiguration('det_dropout'),
                              'dropout_seed': LaunchConfiguration('dropout_seed'),
                              'dropout_mode': LaunchConfiguration('dropout_mode'),
                              'inc_phi_zero': LaunchConfiguration('inc_phi_zero'),
                              'phi_log': LaunchConfiguration('phi_log')}.items()),
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
                          'controller': LaunchConfiguration('controller'),
                          'mpc_rho': ParameterValue(LaunchConfiguration('mpc_rho'), value_type=float),
                          'mpc_sat': ParameterValue(LaunchConfiguration('mpc_sat'), value_type=float),
                          'mpc_horizon': ParameterValue(LaunchConfiguration('mpc_horizon'), value_type=int),
                          'mpc_fit_seed': ParameterValue(LaunchConfiguration('mpc_fit_seed'), value_type=int),
                          'mpc_lam': ParameterValue(LaunchConfiguration('mpc_lam'), value_type=float),
                          'mpc_sub': ParameterValue(LaunchConfiguration('mpc_sub'), value_type=int),
                          'log_csv': f'{LOG_DIR}/koopman.csv'}]),
    ] + bridges + rpm_sims + drift_shims)
