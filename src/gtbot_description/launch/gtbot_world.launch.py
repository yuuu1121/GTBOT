#!/usr/bin/env python3
"""Stonefish 시뮬 기동.

GUI 창 렌더 인자(2026-08-16 노출): window_res_x/y와 rendering_quality는 **화면에
보여주는 창**의 설정이고 센서(LiDAR·관측 카메라)의 발행률·품질과는 별개 경로다.
성능 계측에서 RTF 0.998·odometry 10.00 Hz인데 LiDAR 2.6~7.9 Hz·카메라 1.4~6.7 Hz로
렌더 경로만 미달하는 것이 확인돼, 같은 GPU를 쓰는 창 렌더 부하를 줄일 수 있게
인자로 뺐다. 기본은 1600x900 · low(사용자 지정):

  ros2 launch gtbot_description gtbot_world.launch.py
  # 더 가볍게
  ros2 launch gtbot_description gtbot_world.launch.py window_res_x:=960 window_res_y:=540
  # 종전과 동일하게
  ros2 launch gtbot_description gtbot_world.launch.py \\
      window_res_x:=1280 window_res_y:=720 rendering_quality:=high

**창 크기는 띄울 때만 정해진다 — 마우스 드래그 리사이즈는 안 된다.** Stonefish가 SDL
창을 SDL_WINDOW_RESIZABLE 없이 만들고(GraphicalSimulationApp.cpp:241), 3D 파이프라인에
리사이즈 경로가 아예 없다 — 앱의 windowW/H는 초기화 후 갱신되지 않는데 glViewport·
glScissor에 그대로 쓰이고(같은 파일 1061~1062), 렌더 타깃은 카메라·SSAO·해양·대기 등
여러 파일에 흩어져 초기 크기로 할당된다. RESIZED 이벤트는 gui->Resize()만 불러 GUI
오버레이만 새 크기가 되므로, 플래그만 켜면 3D 화면과 어긋난다.

**미검증**: 창 렌더를 줄이면 센서 발행률이 오르는지는 아직 측정하지 않았다. 이
머신의 측정 산포가 같은 조건에서 1.7배라 단발 비교로는 확인되지 않는다.
rendering_quality가 카메라 센서 이미지 품질에도 걸리는지 역시 미확인 —
관측 영상 화질이 눈에 띄게 나빠지면 그 인자만 high로 되돌릴 것.

물리 스텝(simulation_rate)은 건드리지 말 것. 낮추면 동역학이 바뀌어 지금까지의
게이트 결과가 전부 무효가 된다.

pkrc·bluerov2 깊이 유지(2026-11-01): 씬에서 pkrc는 수심 10 m, bluerov2는 5 m에 스폰하고,
이 런치가 depth_controller_sim을 차량마다 띄워 그 깊이를 바로 유지한다. 끄려면 pkrc_depth:=false,
bluerov2_depth:=false
(pkrc_sim_stack.launch.py를 따로 띄울 때는 둘이 /pkrc/setpoint/pwm를 두고 싸우니 꺼야 한다).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        # Fast DDS 공유메모리 프로파일(2026-08-17). /ouster/points 한 스캔이 512 KiB인데
        # Fast DDS 기본 SHM 세그먼트가 정확히 512 KiB라 이 메시지는 SHM에 못 들어가고
        # UDP로 폴백한다 — 1.5 KB MTU로 ~350 조각이 되어 하나만 잃어도 샘플 전체가
        # 버려진다. 발행은 10.01 Hz인데 구독은 3~5 Hz(C++ 노드는 0.77 Hz)였던 원인이다.
        # 세그먼트를 8 MiB로 키우면 구독이 9.75~10.02 Hz로 실물과 같아진다(실측).
        # 이 컨테이너는 net.core.rmem_max가 sysctl 미노출이라 커널 버퍼 조정은 불가.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', PathJoinSubstitution([
            FindPackageShare('gtbot_description'), 'config', 'fastdds_shm.xml'])),
        DeclareLaunchArgument('scenario', default_value='gtbot_world'),
        DeclareLaunchArgument('simulation_rate', default_value='100.0'),
        # GUI 창 전용 — 센서 경로와 무관. 종전 값은 1280/720/high.
        DeclareLaunchArgument('window_res_x', default_value='1600'),
        DeclareLaunchArgument('window_res_y', default_value='900'),
        DeclareLaunchArgument('rendering_quality', default_value='low'),
        DeclareLaunchArgument('pkrc_depth', default_value='true'),
        DeclareLaunchArgument('pkrc_target_depth', default_value='10.0'),
        DeclareLaunchArgument('bluerov2_depth', default_value='true'),
        DeclareLaunchArgument('bluerov2_target_depth', default_value='5.0'),
    ]
    pkrc_depth = Node(package='pkrc_controller', executable='depth_controller_sim',
                      name='depth_controller_pkrc',
                      output='screen', condition=IfCondition(LaunchConfiguration('pkrc_depth')),
                      parameters=[{'vehicle_name': 'pkrc',
                                   'target_depth': LaunchConfiguration('pkrc_target_depth')}])
    # bluerov2: 스러스터 8기, Dive 4기(인덱스 4~7)가 heave. 부호는 TAM.yaml heave 행 그대로 —
    # 같은 부호를 주면 추력이 상쇄되고 반동 토크만 남아 제자리 회전한다(2026-11-01 실측).
    bluerov2_depth = Node(package='pkrc_controller', executable='depth_controller_sim',
                          name='depth_controller_bluerov2',
                          output='screen', condition=IfCondition(LaunchConfiguration('bluerov2_depth')),
                          parameters=[{'vehicle_name': 'bluerov2',
                                       'target_depth': LaunchConfiguration('bluerov2_target_depth'),
                                       'num_thrusters': 8,
                                       'heave_indices': [4, 5, 6, 7],
                                       'heave_signs': [-1.0, 1.0, 1.0, -1.0],
                                       # 부력 상쇄 정상 PWM 0.49 > 기본 한계 1.0×0.35(실측) → 적분 한계 4.0.
                                       # ki 0.35면 5 m 주위 ±0.1 m 진동, 0.15면 ±0.05 m(실측 2026-11-01).
                                       'ki': 0.15,
                                       'max_integral': 4.0}])
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('stonefish_ros2'), '/launch/simulator_gpu.launch.py']),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([FindPackageShare('gtbot_description'), '']),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('gtbot_description'), 'scenarios',
                [LaunchConfiguration('scenario'), '.scn']]),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'window_res_x': LaunchConfiguration('window_res_x'),
            'window_res_y': LaunchConfiguration('window_res_y'),
            'rendering_quality': LaunchConfiguration('rendering_quality'),
        }.items())
    return LaunchDescription(args + [simulator, pkrc_depth, bluerov2_depth])
