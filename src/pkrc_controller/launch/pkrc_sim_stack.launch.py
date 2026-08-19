"""PKRC 시뮬 스택 — 편대 아래 수심 4 m 유지 + upward 카메라 마커 위치추정.

구성 (gtbot_world.scn의 pkrc 로봇 전제):
  depth_controller_sim   /pkrc/pressure -> 깊이 환산, PID -> /pkrc/setpoint/pwm heave(4·5번)
                         목표는 target_depth 파라미터 또는 /pkrc/depth/target (teleop_depth로 조절 가능)
  aruco_detector_6dof    /pkrc/up/image_color 에서 로봇 하부 aruco LED 검출 -> /aruco/pose_array
  ukfm_localization      IMU(/pkrc/imu) + 깊이(/pkrc/depth/current) + aruco 융합 -> /ukfm/odom
                         dvl_msgs 부재 시 DVL 융합은 자동 비활성(가드 임포트)

마커 맵은 편대 스폰 시점의 LED 월드 위치(정지 기준). 로봇이 움직이면 맵이 어긋나므로
주행 중 절대위치가 아니라 '마커에 대한 상대 위치' 관찰이 목적일 때 쓸 것.
  id 8=gtbot(1.5,0)  id 17=gtbot2(-0.75,1.3)  id 58=gtbot3(-0.75,-1.3)  id 59=platform(0,0)
  (LED 오프셋 y-0.138, z는 수면 위 ~0.5 m — scn의 사용자 배치 그대로)

사용: ros2 launch pkrc_controller pkrc_sim_stack.launch.py [target_depth:=4.0]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MARKER_IDS = [8, 17, 58, 59]
MAP_X = [1.5, -0.75, -0.75, 0.0]
MAP_Y = [-0.138, 1.162, -1.438, 0.0]
MAP_Z = [-0.53, -0.53, -0.53, -0.52]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_depth', default_value='4.0'),
        Node(package='pkrc_controller', executable='depth_controller_sim',
             output='screen',
             parameters=[{'vehicle_name': 'pkrc',
                          'target_depth': LaunchConfiguration('target_depth')}]),
        Node(package='pkrc_controller', executable='aruco_detector_6dof',
             output='screen',
             parameters=[{'use_direct_camera': False,
                          'image_topic': '/pkrc/up/image_color',
                          'camera_info_topic': '/pkrc/up/camera_info',
                          'marker_ids': MARKER_IDS,
                          # 기하 게이트(2026-08-19 4차 데모): 수심 4 m·FOV 82°에서 유효
                          # 슬랜트는 최대 ~5.4 m. Z>5.5 검출은 수면 내부전반사 반사상 등
                          # 허위 양성(거리 7~14 m에서도 ID 17·59가 '검출'됐다) — 기각한다.
                          'max_depth': 5.5,
                          'marker_map_ids': MARKER_IDS,
                          'marker_map_x': MAP_X,
                          'marker_map_y': MAP_Y,
                          'marker_map_z': MAP_Z}]),
        Node(package='pkrc_controller', executable='ukfm_localization',
             output='screen',
             parameters=[{'imu_topic': '/pkrc/imu',
                          # 실기 기본값 imu_inverted=True는 뒤집혀 장착된 실물 IMU 보정 —
                          # 시뮬 IMU는 정방향이라 끈다(켜면 중력 적분이 틀어져 z가 발산)
                          'imu_inverted': False,
                          # ukfm의 pressure_callback은 mbar 절대압 기대(1013.25를 빼고
                          # 환산) — depth_controller_sim이 중계하는 mbar 토픽을 물린다.
                          # 깊이(m)를 물리면 z가 −10 m에 박힌다(실측).
                          'pressure_topic': '/pkrc/pressure_mbar',
                          'aruco_topic': '/aruco/pose_array',
                          'use_dvl': False,
                          'marker_map_ids': MARKER_IDS,
                          'marker_map_x': MAP_X,
                          'marker_map_y': MAP_Y,
                          'marker_map_z': MAP_Z}]),
    ])
