"""platform LiDAR → gtbot별 상대위치·헤딩 추정 → /gtbotN/state_est.
rel_*는 platform 기준·월드축 정렬(IMU yaw 회전), yaw는 월드 기준.

프레임 규약은 `perception_core.lidar_to_world` 도크스트링 참조(round 4 실측으로 확정:
클라우드는 FLU/z-위, 월드 정렬은 R(+yaw)). z_water_offset = 라이다의 수면 위 높이
(마운트 0.36 m − platform 흘수 0.162 m = 0.198 m) — 흘수가 바뀌면 여기만 다시 잰다."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of
from .perception_core import (cluster_2d, lidar_to_world, marker_split, marker_heading,
                              marker_center, kf_step)
from .relative_state import OFFSETS

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

SPAWN_REL = [(1.5, 0.0), (-0.75, 1.3), (-0.75, -1.3)]  # 스폰 상대 위치 — 트랙 초기값·재획득 앵커
# fix round 2: 3.6 m 스폰(구 (3,±2))에서 yaw_meas 오차 40~50°(마커 유효거리 밖, aux 포인트
# 부족으로 aux 센트로이드 노이즈 큼) 실측 → 전 로봇 1.5 m 반경(gtbot 2 m 실측 시 오차 8.6°와
# 동급)으로 재배치. 편대 간격(0.87 m 등변삼각형, 변길이 2.6 m)은 유지.

class PlatformPerception(Node):
    def __init__(self):
        super().__init__('platform_perception')
        for n, d in [('z_water_offset', 0.198),
                     ('linkage', 0.3), ('track_gate', 0.6), ('n_accum', 10),
                     ('min_sep', 0.08), ('main_band', [0.0, 0.07]), ('shared_band', [0.10, 0.22]),
                     ('kf_q', 0.5), ('kf_r', 0.025), ('marker_offset', [0.0, -0.10])]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.z_off = p('z_water_offset')
        self.linkage, self.gate = p('linkage'), p('track_gate')
        self.n_accum = int(p('n_accum'))
        self.min_sep = p('min_sep')
        self.main_band = tuple(p('main_band'))
        self.shared_band = tuple(p('shared_band'))
        self.buf = []                                   # (pts_world_xy, h) 누적 버퍼 (비반복 스캔 병합)
        self.yaw_p = 0.0
        # 초기 트랙 = 스폰 상대 위치(로봇들은 편대 밖에서 출발 — Task 1 실측 반영)
        self.tracks = [np.array(o) for o in SPAWN_REL]
        self.kf_q, self.kf_r = p('kf_q'), p('kf_r')
        self.marker_offset = tuple(p('marker_offset'))   # 마스트 중점의 body 위치(캘리브레이션)
        self.kf = [None] * 3                            # k -> (x=[px,py,vx,vy], P, t) 트랙별 KF
        self.all_invalid_since = None                   # fix round 1: 전 트랙 invalid 5s 지속 → 스폰 리셋
        self.create_subscription(Imu, '/platform/imu', self.on_imu, 10)
        self.create_subscription(PointCloud2, '/platform/lidar/points', self.on_cloud, 5)
        self.pubs = [self.create_publisher(Float64MultiArray, f'/{r}/state_est', 10)
                     for r in ROBOTS]

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw_p = yaw_of(q.x, q.y, q.z, q.w)

    def on_cloud(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        pts = np.array([[q[0], q[1], q[2]] for q in
                        point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)])
        out = [None] * 3
        if len(pts):
            xy_w, h = lidar_to_world(pts, self.yaw_p, self.z_off)   # 수신 시점 yaw로 월드축 정렬
            r = np.hypot(pts[:, 0], pts[:, 1])
            m = (h > 0.2) & (h < 0.55) & (r > 0.3) & (r < 5.0)  # 마커 대역만 (스폰 3.6 m 커버)
            if m.any():                                         # 선체 상단은 0.147 m — 대역 밖
                self.buf.append((xy_w[m], h[m]))
        self.buf = self.buf[-self.n_accum:]
        if self.buf:
            xy_w = np.vstack([b[0] for b in self.buf])          # 비반복 스캔 병합 → 유효 해상도 증가
            h_w = np.concatenate([b[1] for b in self.buf])
            xyz_w = np.column_stack([xy_w, h_w])
            for idx in cluster_2d(xy_w, self.linkage):
                sp = marker_split(xyz_w[idx], h_w[idx], self.main_band, self.shared_band, self.min_sep)
                if sp is None:
                    continue
                center_xy, aux_xy = sp
                # 두 기둥은 body (±0.08, -0.10) — 중점이 body (0,-0.10)이라 헤딩으로 되돌려야
                # 로봇 원점이 된다(보정 없으면 0.10 m 상수 편향).
                rel_w = marker_center(center_xy, aux_xy, self.marker_offset)
                yaw_w = marker_heading(center_xy, aux_xy)
                k = int(np.argmin([np.linalg.norm(rel_w - tr) for tr in self.tracks]))
                if np.linalg.norm(rel_w - self.tracks[k]) < self.gate and out[k] is None:
                    out[k] = (rel_w, yaw_w)
        # fix round 1: 전 트랙이 동시에 5s 넘게 invalid면 스폰 상대위치로 리셋(드리프트로
        # 트랙이 영구 미아가 되는 것을 막는 재획득 앵커 — 실측: 이 게이트 없이는 valid_ratio가
        # 스폰 후 몇 분 뒤 0으로 붕괴함, task-3-report 참조).
        if all(o is None for o in out):
            if self.all_invalid_since is None:
                self.all_invalid_since = t
            elif t - self.all_invalid_since > 5.0:
                self.tracks = [np.array(o) for o in SPAWN_REL]
        else:
            self.all_invalid_since = None
        for k, pub in enumerate(self.pubs):
            if out[k] is None:
                pub.publish(Float64MultiArray(data=[0.0] * 5 + [0.0]))
                continue
            rel_w, yaw_w = out[k]
            # 트랙별 상수속도 KF — 발행 위치·속도는 필터 상태다(유한차분 대체). 실종 구간에는
            # 전파하지 않고(무효 발행), 재획득 시 실제 경과 dt로 한 스텝 돌려 자연히 이어붙인다.
            dt = t - self.kf[k][2] if self.kf[k] is not None else None
            if dt is None or not (0.0 < dt < 1.0):              # 초기화·긴 공백 -> 재초기화
                x = np.array([rel_w[0], rel_w[1], 0.0, 0.0])
                P = np.diag([self.kf_r ** 2, self.kf_r ** 2, 0.5 ** 2, 0.5 ** 2])
            else:
                x, P = kf_step(self.kf[k][0], self.kf[k][1], rel_w, dt, self.kf_q, self.kf_r)
            self.kf[k] = (x, P, t)
            self.tracks[k] = rel_w                              # 게이트 앵커는 생측정 유지(트래킹 시맨틱 불변)
            pub.publish(Float64MultiArray(
                data=[float(x[0]), float(x[1]), float(x[2]), float(x[3]),
                      float(yaw_w), 1.0]))

def main():
    rclpy.init()
    rclpy.spin(PlatformPerception())
