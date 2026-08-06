"""platform LiDAR → gtbot별 상대위치·헤딩 추정 → /gtbotN/state_est.
rel_*는 platform 기준·월드축 정렬(IMU yaw 회전), yaw는 월드 기준.
z_water_offset·z_sign 기본값은 Task 1 프로브 실측(0.198) + fix round 1 h_max 잔차 보정(-0.02)
으로 채운다(diag_h_hist.py 실측: main mast top h_max≈0.399, 설계값 0.38 대비 +0.019 과다 →
z_water_offset을 그만큼 낮춰 상쇄). marker_split은 h_max 기준 상대 밴딩이라 이 보정에
민감하지 않지만, 대역 사전필터(h 0.2~0.55)는 절대 기준이라 반영."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of
from .perception_core import cluster_2d, marker_split, marker_heading
from .relative_state import OFFSETS

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

SPAWN_REL = [(2.0, 0.0), (3.0, 2.0), (3.0, -2.0)]      # 스폰 상대 위치 — 트랙 초기값·재획득 앵커

class PlatformPerception(Node):
    def __init__(self):
        super().__init__('platform_perception')
        for n, d in [('z_water_offset', 0.179), ('z_sign', -1.0),
                     ('linkage', 0.3), ('track_gate', 0.6), ('n_accum', 5),
                     ('min_sep', 0.08), ('main_band', [0.0, 0.05]), ('shared_band', [0.06, 0.14])]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.z_off, self.z_sign = p('z_water_offset'), p('z_sign')
        self.linkage, self.gate = p('linkage'), p('track_gate')
        self.n_accum = int(p('n_accum'))
        self.min_sep = p('min_sep')
        self.main_band = tuple(p('main_band'))
        self.shared_band = tuple(p('shared_band'))
        self.buf = []                                   # (pts_world_xy, h) 누적 버퍼 (비반복 스캔 병합)
        self.yaw_p = 0.0
        # 초기 트랙 = 스폰 상대 위치(로봇들은 편대 밖에서 출발 — Task 1 실측 반영)
        self.tracks = [np.array(o) for o in SPAWN_REL]
        self.prev = [None] * 3                          # (rel_xy, t) — 속도 차분용
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
            h = (pts[:, 2] - self.z_off) * self.z_sign          # 수면 위 높이
            r = np.hypot(pts[:, 0], pts[:, 1])
            m = (h > 0.2) & (h < 0.55) & (r > 0.3) & (r < 5.0)  # 마커 대역만 (스폰 3.6 m 커버)
            sel, hsel = pts[m], h[m]
            if len(sel):
                c, s = np.cos(self.yaw_p), np.sin(self.yaw_p)
                R = np.array([[c, -s], [s, c]])                 # 센서(≈platform body)→월드축, 수신 시점 yaw
                # Task 1 실측: Stonefish orientation은 world→body 컨벤션 — body→world 변환은
                # R.T가 아니라 R을 그대로(전치 없이) 적용해야 함(실측 교차검증, task-3-report 참조).
                self.buf.append((sel[:, :2] @ R, hsel))         # 월드축 정렬 후 누적
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
                rel_w = np.asarray(center_xy)
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
            v = np.zeros(2)
            if self.prev[k] is not None:
                prel, pt = self.prev[k]
                dt = t - pt
                if 0.0 < dt < 1.0:                              # 누적 주기 고려 상한 완화
                    v = (rel_w - prel) / dt
            self.prev[k] = (rel_w, t)
            self.tracks[k] = rel_w
            pub.publish(Float64MultiArray(
                data=[float(rel_w[0]), float(rel_w[1]), float(v[0]), float(v[1]),
                      float(yaw_w), 1.0]))

def main():
    rclpy.init()
    rclpy.spin(PlatformPerception())
