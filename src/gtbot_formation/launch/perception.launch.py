"""지각 스택만 기동 — ouster_cluster(실물 랩 검출) + platform_perception(브리지).
게이트 S4(정지 지각 품질)는 편대 제어 없이 이 launch + gate_s4로 판정한다.
formation.launch.py가 이 파일을 include해 파라미터 중복을 없앤다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 저하 시나리오 주입(기본 0 = 종전 동작). 스윕 러너가 값을 바꿔가며 게이트를 돌린다.
    bias = ParameterValue(LaunchConfiguration('lidar_yaw_bias'), value_type=float)
    dropout = ParameterValue(LaunchConfiguration('det_dropout'), value_type=float)
    dseed = ParameterValue(LaunchConfiguration('dropout_seed'), value_type=int)
    dmode = LaunchConfiguration('dropout_mode')
    pzero = ParameterValue(LaunchConfiguration('inc_phi_zero'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('lidar_yaw_bias', default_value='0.0'),
        DeclareLaunchArgument('det_dropout', default_value='0.0'),
        # 반복 런이 같은 잡음 궤적을 반복하지 않도록 시드를 런마다 바꾼다
        DeclareLaunchArgument('dropout_seed', default_value='0'),
        # 'incidence'면 입사각 의존 검출(실물 근거) — 기본은 종전 'uniform'
        DeclareLaunchArgument('dropout_mode', default_value='uniform'),
        # 실측 수준 곡선을 걸 수 있게 노출(기본 50 = 순한 가정)
        DeclareLaunchArgument('inc_phi_zero', default_value='50.0'),
        # 실물 랩 검출 파이프라인(gtbot_lidar_cluster) — 파라미터는 랩 launch의 실기
        # 튜닝값 그대로, 시뮬 기하에 따른 오버라이드 3개만 다르다:
        #   input_topic: 시뮬 OS0 토픽 / preprocess_max_range 3→5 (스폰 1.5 m + S6 과도
        #   이탈 여유, 구 마스트 파이프라인의 r<5와 동일) / z 크롭: 판이 센서 기준
        #   -0.053~+0.047 m(scn 주석 참조)라 [-0.5,0]→[-0.15,+0.10].
        Node(package='ouster_cluster', executable='ouster_cluster_node',
             name='ouster_cluster_node',
             parameters=[{
                 'input_topic': '/ouster/points',  # ouster-ros 규약(드라이버 기본값) — 실기·시뮬 동일 설정
                 'preprocess_max_range': 5.0,
                 # z-크롭 재설계(2026-08-10): platform 롤/피치 요동(±2~3°)이 거리 r에서
                 # 창을 ±0.04·r 휩쓸어 판이 프레임별로 들락거림(리플레이 실측: z-band가
                 # 프레임 간 6 cm 점프, confirm 3연속 불가). 판을 10 cm 추가 리프트해
                 # 센서 기준 +0.07~+0.17에 두고 상한을 열어 스윕을 흡수. 선체 림(-0.083)은
                 # 하한 -0.06이 계속 배제.
                 # 판 14x20cm(sim 보정, 2026-08-10): 10cm 판은 1.5~3m에서 링 3~4개만
                 # 걸려 측정 z-밴드(0.06~0.07)가 물리 높이(0.10)에 항상 미달, 높이
                 # 게이트가 구조적으로 기각(리플레이 실측). 실물은 반사강도로 잡지만
                 # sim은 기하 게이트뿐이라 수직 확장으로 링 샘플링을 배가 — BEV
                 # 발자국(14cm 폭)과 yaw 기하는 불변. simplified: 실기 대조 시 재보정.
                 'preprocess_z_min': -0.25,  # 강도 컷 도입으로 넓게: 판 밴드(-0.14~-0.04)+여유
                 'preprocess_z_max': 0.10,
                 # 반사강도 컷(2026-08-10): sim RotatingLidar가 판 명중에 255, 배경 1을
                 # 발행(실물 레트로리플렉터 동등) — 선체 클러터를 재질로 분리, 기하
                 # 게이트 부담 해소. 실기에서도 reflectivity 채널로 동일 사용 가능.
                 'preprocess_min_intensity': 100.0,
                 'preprocess_voxel_leaf': 0.0,
                 'elev_filter_enable': True,
                 'elev_min_deg': -30.0,
                 'elev_max_deg': 25.0,  # sim 보정: 판 상단(센서 위 0.37m)이 근거리 1.4m에서 15° 컷에 잘림
                 'bev_density_cell_size': 0.02,
                 'bev_density_min': 3.0,  # sim 보정: 시뮬 OS0 판 셀 밀도 실측 5~27 (랩 벤치 30~300)
                 'bev_density_max': 0.0,
                 'bev_density_z_bin': 0.05,
                 'bev_density_min_pts_per_bin': 1,  # sim 보정: 시뮬 링 밀도
                 'bev_density_min_z_bins': 2,
                 'bev_density_dual_grid': True,
                 'bev_density_drop_lonely': True,
                 'bev_line_min_cells': 5,
                 'bev_line_max_width': 0.09,
                 'bev_line_min_length': 0.05,
                 'bev_line_max_length': 0.25,
                 'bev_line_z_gap': 0.10,
                 'bev_line_diag_flank_gate': True,
                 'bev_line_touch_connect': False,  # sim 보정: 희소 링에서 다수결 매칭이 셀 연결을 끊음 — 내장 median-z 폴백 사용
                 'bev_line_touch_gap': 0.015,
                 'bev_line_touch_xy_gap': 0.03,
                 'bev_line_touch_min_matched': 4,
                 'bev_line_touch_min_ratio': 0.6,
                 'bev_line_z_med_gap': 0.04,
                 'bev_line_top_tol': 0.10,
                 'bev_line_top_step': 0.07,
                 'bev_plate_width': 0.14,
                 'bev_plate_height': 0.10,
                 'bev_plate_tol_width': 0.05,
                 'bev_plate_tol_height': 0.05,  # sim 링 샘플링: 실측 밴드 0.06~0.07
                 'bev_plate_thickness_max': 0.0,
                 'bev_line_max_segments': 50,
                 'bev_refine_enable': False,
                 'bev_refl_enable': False,
                 'bev_seg_match_distance': 0.15,
                 'bev_seg_confirm_frames': 2,  # sim 보정: 부유 표적 히브로 검출 깜빡임
                 'bev_seg_max_missed': 10,  # sim 보정: 동일
                 'plate_recover_enable': True,
                 'plate_recover_xy_half': 0.12,
                 'plate_recover_z_margin_below': 0.0,
                 'plate_recover_z_margin_above': 0.02,
                 'plate_recover_tolerance': 0.03,
                 'plate_recover_min_points': 10,
                 'plate_recover_maha_threshold': 7.815,
                 'plate_recover_accum_frames': 3,  # sim 보정: 히브 재위상으로 링 간극 메움
                 'plate_thickness_max': 0.12,  # sim 보정: 록킹 틸트 시 20cm 판 투영두께 0.02+0.20sinθ  # sim 보정: 노이즈 sigma=1cm의 면 두께 퍼짐 실측 3.6~5.4cm (38/38 이 게이트 단독 탈락)
                 'plate_width': 0.14,
                 'plate_height': 0.10,
                 'plate_size_tol': 0.05,
                 'plate_min_points': 10,
                 'track_match_distance': 0.5,
                 'track_pos_alpha': 0.5,
                 'track_vel_alpha': 0.5,
                 'track_yaw_alpha': 0.2,
                 'track_confirm_frames': 2,
                 'track_max_missed': 6,
                 'track_max_dt': 0.3,
                 'scalar_field': 'intensity',
             }]),
        Node(package='gtbot_formation', executable='platform_perception',
             # sim: 반사강도 클러스터링 직접 검출(points). 실기 전환 시 'boxes'로
             # 바꾸면 ouster_cluster(실물 검출기) 출력을 소비한다 — 하류 동일.
             parameters=[{'input_mode': 'points', 'intensity_min': 100.0,
                          'lidar_yaw_bias': bias, 'det_dropout': dropout,
                          'dropout_seed': dseed, 'dropout_mode': dmode, 'inc_phi_zero': pzero}]),
    ])
