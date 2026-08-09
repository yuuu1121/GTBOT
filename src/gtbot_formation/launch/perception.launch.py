"""지각 스택만 기동 — ouster_cluster(실물 랩 검출) + platform_perception(브리지).
게이트 S4(정지 지각 품질)는 편대 제어 없이 이 launch + gate_s4로 판정한다.
formation.launch.py가 이 파일을 include해 파라미터 중복을 없앤다."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 실물 랩 검출 파이프라인(gtbot_lidar_cluster) — 파라미터는 랩 launch의 실기
        # 튜닝값 그대로, 시뮬 기하에 따른 오버라이드 3개만 다르다:
        #   input_topic: 시뮬 OS0 토픽 / preprocess_max_range 3→5 (스폰 1.5 m + S6 과도
        #   이탈 여유, 구 마스트 파이프라인의 r<5와 동일) / z 크롭: 판이 센서 기준
        #   -0.053~+0.047 m(scn 주석 참조)라 [-0.5,0]→[-0.15,+0.10].
        Node(package='ouster_cluster', executable='ouster_cluster_node',
             name='ouster_cluster_node',
             parameters=[{
                 'input_topic': '/platform/lidar/points',
                 'preprocess_max_range': 5.0,
                 'preprocess_z_min': -0.06,
                 'preprocess_z_max': 0.10,
                 'preprocess_voxel_leaf': 0.0,
                 'elev_filter_enable': True,
                 'elev_min_deg': -30.0,
                 'elev_max_deg': 15.0,
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
                 'bev_plate_tol_height': 0.04,
                 'bev_plate_thickness_max': 0.0,
                 'bev_line_max_segments': 50,
                 'bev_refine_enable': False,
                 'bev_refl_enable': False,
                 'bev_seg_match_distance': 0.15,
                 'bev_seg_confirm_frames': 3,  # sim 보정: 부유 표적 히브로 검출 깜빡임
                 'bev_seg_max_missed': 5,  # sim 보정: 동일
                 'plate_recover_enable': True,
                 'plate_recover_xy_half': 0.12,
                 'plate_recover_z_margin_below': 0.0,
                 'plate_recover_z_margin_above': 0.02,
                 'plate_recover_tolerance': 0.03,
                 'plate_recover_min_points': 10,
                 'plate_recover_maha_threshold': 7.815,
                 'plate_recover_accum_frames': 3,  # sim 보정: 히브 재위상으로 링 간극 메움
                 'plate_thickness_max': 0.03,
                 'plate_width': 0.14,
                 'plate_height': 0.10,
                 'plate_size_tol': 0.03,
                 'plate_min_points': 10,
                 'track_match_distance': 0.5,
                 'track_pos_alpha': 0.5,
                 'track_vel_alpha': 0.5,
                 'track_yaw_alpha': 0.2,
                 'track_confirm_frames': 2,
                 'track_max_missed': 3,
                 'track_max_dt': 0.3,
                 'scalar_field': 'intensity',
             }]),
        Node(package='gtbot_formation', executable='platform_perception'),
    ])
