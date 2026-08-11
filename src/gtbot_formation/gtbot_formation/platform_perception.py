"""ouster_cluster 반사판 검출(boxes) → gtbot별 상대위치·헤딩 추정 → /gtbotN/state_est.
rel_*는 platform 기준·월드축 정렬(IMU yaw 회전), yaw는 월드 기준.

검출 스테이지는 실물 랩 파이프라인(gtbot_lidar_cluster의 ouster_cluster_node,
OS0 클라우드 → BEV 라인 → 14×10cm plate 판정 → EMA 트래커 → /ouster_cluster/boxes)이
담당하고, 이 노드는 boxes를 platform 상대·월드 정렬로 변환해 트랙 배정·KF·발행만
한다(연관/재획득/KF 로직은 마스트 캠페인에서 검증된 것을 그대로 재사용).

프레임 규약: boxes는 입력 클라우드 프레임(FLU, z 위) — 중심 변환은
`perception_core.lidar_to_world`(y 미러 + R(+yaw_p)) 재사용, 방향각은 FLU y 미러로
θ→−θ이므로 world = yaw_p − θ_flu. z_water_offset = 라이다의 수면 위 높이
(마운트 0.36 m − platform 흘수 0.162 m = 0.198 m) — 흘수가 바뀌면 여기만 다시 잰다.

plate yaw는 π-대칭(mod 180°)이라 로봇 헤딩은 `fold_heading`이 베어링 사전정보
(팔로워는 platform 지향)로 접어 해소한다 — 기대 헤딩에서 90° 넘게 벗어난 자세는
원리적으로 복원 불가(마커 체계의 구조적 성질, S4 이탈로 문서화)."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of
from .perception_core import (lidar_to_world, decode_wirebox, fold_heading,
                              kf_step, assign_tracks)

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

SPAWN_REL = [(1.5, 0.0), (-0.75, 1.3), (-0.75, -1.3)]  # 스폰 상대 위치 — 트랙 초기값·재획득 앵커

class PlatformPerception(Node):
    def __init__(self):
        super().__init__('platform_perception')
        for n, d in [('z_water_offset', 0.198),
                     ('track_gate', 0.6),
                     ('kf_q', 0.5), ('kf_r', 0.025),
                     # marker_offset(2026-08-11): 판 중심의 body 좌표. 라이다는 판을
                     # 보는데 제어·평가는 로봇 원점 기준이라, 이 값을 빼지 않으면 판까지의
                     # 거리가 그대로 위치 오차가 된다. 판이 충돌체로 들어온 뒤 실측 위치
                     # RMSE가 세 대 모두 0.138 m로 나왔고, 이는 판 중심 오프셋
                     # |(0.0014, -0.1386)| = 0.1386 m와 일치한다(S4 임계 0.1 m 초과의 원인).
                     # 값은 GTBOT.obj 원본 메시에서 실측한 판 중심(몸체 프레임).
                     ('marker_offset', [0.0014, -0.1386]),
                     ('track_reset', 4.0),
                     ('input_mode', 'boxes'), ('intensity_min', 100.0),
                     # yaw_se_max(2026-08-10, 프로브 D): 라인피팅 기울기 표준오차(rad)가
                     # 이 값을 넘는 프레임은 헤딩 EMA 갱신 보류(위치·KF는 정상). est 체제
                     # 실측: 헤딩 오차가 요레이트 상관(+0.25~0.41) — 요동으로 판이
                     # 비스듬해진 순간 교차-거리 스팬이 줄어 기울기 분산이 폭발하는
                     # 아스펙트 붕괴 프레임의 선별 차단. 0=게이트 없음.
                     # 0.12(프로브 D): S4 첫 통과(7.7/7.8/10.0°). 0.08(프로브 E)은
                     # 역효과(S4 여유 축소·S6 열화) — 0.12 확정. S5는 GT 물리 정렬
                     # 지표라 이 게이트와 무관(gate_s5.py).
                     ('yaw_se_max', 0.12)]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.z_off = p('z_water_offset')
        self.gate = p('track_gate')
        self.yaw_p = 0.0
        # 초기 트랙 = 스폰 상대 위치(로봇들은 편대 밖에서 출발 — Task 1 실측 반영)
        self.tracks = [np.array(o) for o in SPAWN_REL]
        self.kf_q, self.kf_r = p('kf_q'), p('kf_r')
        self.marker_offset = np.array(p('marker_offset'))  # 판 중심의 body 위치(캘리브레이션)
        self.track_reset = p('track_reset')              # 트랙별 재획득 대기(초)
        self.invalid_since = [None] * 3                 # k -> 그 트랙이 무효로 들어간 시각
        self.kf = [None] * 3                            # k -> (x=[px,py,vx,vy], P, t) 트랙별 KF
        self.yaw_ema = [None] * 3                       # k -> 평활 헤딩(코스팅 발행에도 사용)
        self.all_invalid_since = None                   # 전 트랙 invalid 5s 지속 → 스폰 리셋
        self.create_subscription(Imu, '/platform/imu', self.on_imu, 10)
        # input_mode(2026-08-10): 'boxes' = 실물 랩 검출기(ouster_cluster) 출력 소비
        # (실기 경로). 'points' = 반사강도 클러스터링 직접 검출(sim 경로) — sim
        # RotatingLidar의 판=255 강도 모델 위에서 클러스터+PCA로 중심·장축을 얻는다.
        # 실물도 레트로리플렉터 강도 컷 기반이라 검출 원리는 동일하고, 하류(연관·
        # KF·발행)는 두 모드가 완전히 공유한다.
        self.input_mode = p('input_mode')
        self.intensity_min = float(p('intensity_min'))
        self.yaw_se_max = float(p('yaw_se_max'))
        if self.input_mode == 'points':
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(PointCloud2, '/ouster/points', self.on_points, 5)
        else:
            self.create_subscription(MarkerArray, '/ouster_cluster/boxes', self.on_boxes, 5)
        self.pubs = [self.create_publisher(Float64MultiArray, f'/{r}/state_est', 10)
                     for r in ROBOTS]

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw_p = yaw_of(q.x, q.y, q.z, q.w)

    def _det_from_flu(self, center_flu, line_yaw_flu, yaw_ok=True):
        """라이다 FLU (중심, 장축각) -> world (마커오프셋 보정 원점, 접힌 헤딩, 헤딩 신뢰)."""
        xy_w, _h = lidar_to_world(np.array([center_flu]), self.yaw_p, self.z_off)
        xy = xy_w[0]
        line_yaw_w = self.yaw_p - line_yaw_flu           # FLU y 미러: θ→−θ, 이후 R(+yaw_p)
        expected = np.arctan2(-xy[1], -xy[0])            # 베어링 사전정보: platform 지향
        heading = fold_heading(line_yaw_w, expected)
        c, s = np.cos(heading), np.sin(heading)
        origin = xy - np.array([[c, -s], [s, c]]) @ self.marker_offset
        return (origin, heading, yaw_ok)

    def on_boxes(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        dets = []
        for m in msg.markers:
            if m.id < 0 or len(m.points) != 24:          # DELETEALL 클리어 마커 등 제외
                continue
            pts = [(q.x, q.y, q.z) for q in m.points]
            center_flu, line_yaw_flu = decode_wirebox(pts)
            dets.append(self._det_from_flu(center_flu, line_yaw_flu))
        self._process(dets, t)

    def on_points(self, msg):
        import sensor_msgs_py.point_cloud2 as pc2
        t = self.get_clock().now().nanoseconds * 1e-9
        arr = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z', 'intensity'),
                                    skip_nans=True)
        hi = arr[arr[:, 3] >= self.intensity_min]
        dets = []
        if len(hi) >= 5:
            xy = hi[:, :2].astype(float)
            # 탐욕 클러스터링: 판 간 최소 간격(스폰 1.1 m+) >> 판 폭(0.14 m)이라
            # 반경 0.35 m 단순 배정으로 충분하다.
            centers, members = [], []
            for q in xy:
                for i, c in enumerate(centers):
                    if np.hypot(q[0] - c[0], q[1] - c[1]) < 0.35:
                        members[i].append(q)
                        centers[i] = np.mean(members[i], axis=0)
                        break
                else:
                    centers.append(q.copy())
                    members.append([q])
            for c, m in zip(centers, members):
                if len(m) < 5:                            # 잡음 클러스터 배제
                    continue
                pts2 = np.array(m) - c
                # 광선-회귀 적합(2026-08-10): 라이다 거리 노이즈는 광선 방향에만
                # 있고 방위각은 무잡음이다. 원시 PCA는 노이즈 분산(광선 방향)이 판
                # 길이 분산과 비슷해 주축이 광선 쪽으로 회전(S4 실측 15° 계통 편향).
                # 교차-거리(x, 무잡음)에 대한 광선-거리(y)의 OLS 회귀는 이 편향이
                # 없다(y-잡음만 가정하는 회귀의 기본 성질). 2σ 트림 1회로 모서리
                # 리턴 지렛대 억제. 오프라인 실측: 중앙값 15°→8.8°.
                u_ray = c / max(float(np.hypot(c[0], c[1])), 1e-6)
                u_perp = np.array([-u_ray[1], u_ray[0]])
                x = pts2 @ u_perp
                y = pts2 @ u_ray
                s = float(np.polyfit(x, y, 1)[0])
                res = y - s * x
                keep = np.abs(res - np.median(res)) < 2 * np.std(res) + 1e-9
                if keep.sum() >= 5:
                    s = float(np.polyfit(x[keep], y[keep], 1)[0])
                    x, y = x[keep], y[keep]
                # 기울기 표준오차(rad 근사) — 아스펙트 붕괴 프레임 선별(yaw_se_max 주석)
                sx = float(np.std(x))
                se = float(np.std(y - s * x)) / max(sx * np.sqrt(len(x)), 1e-6)
                yaw_ok = self.yaw_se_max <= 0.0 or se <= self.yaw_se_max
                dvec = u_perp + s * u_ray
                line_yaw_flu = float(np.arctan2(dvec[1], dvec[0]))
                z_mid = float(np.median(hi[:, 2]))    # 판 높이는 전 로봇 공통
                dets.append(self._det_from_flu((float(c[0]), float(c[1]), z_mid),
                                               line_yaw_flu, yaw_ok))
        self._process(dets, t)

    def _process(self, dets, t):
        out = [None] * 3
        # 전역 최적 배정(탐욕 선점 기아 제거, round 12). 게이트 0.6은 그대로 강제된다.
        asg = assign_tracks([d[0] for d in dets], self.tracks, self.gate)
        for k, j in enumerate(asg):
            if j is not None:
                out[k] = dets[j]
        # 트랙별 재획득: 어떤 트랙이 track_reset초 연속 무효면 그 트랙만 되살린다. 미배정
        # 검출이 있으면 그쪽으로 앵커를 스냅하고(고착 해제의 근본), 없으면 스폰 규약으로.
        free = [j for j in range(len(dets)) if j not in asg]
        for k in range(3):
            if out[k] is not None:
                self.invalid_since[k] = None
                continue
            if self.invalid_since[k] is None:
                self.invalid_since[k] = t
            elif t - self.invalid_since[k] > self.track_reset:
                if free:
                    j = min(free, key=lambda q: np.linalg.norm(dets[q][0] - self.tracks[k]))
                    free.remove(j)
                    self.tracks[k] = dets[j][0]
                else:
                    self.tracks[k] = np.array(SPAWN_REL[k])
                self.invalid_since[k] = None
        # 전 트랙이 동시에 5s 넘게 invalid면 스폰 상대위치로 리셋(드리프트로 트랙이
        # 영구 미아가 되는 것을 막는 재획득 앵커).
        if all(o is None for o in out):
            if self.all_invalid_since is None:
                self.all_invalid_since = t
            elif t - self.all_invalid_since > 5.0:
                self.tracks = [np.array(o) for o in SPAWN_REL]
        else:
            self.all_invalid_since = None
        for k, pub in enumerate(self.pubs):
            if out[k] is None:
                # 코스팅(2026-08-10): 짧은 검출 공백(<0.8 s)은 상수속도 KF 예측을
                # 유효로 발행해 잇는다 — sim의 록킹 유도 프레임별 명멸(per-frame
                # 30~50%)이 est 유효율을 붕괴시키는 것을 추정기 표준 설계로 흡수.
                # 헤딩은 마지막 평활값 유지. 긴 공백은 기존대로 무효.
                if self.kf[k] is not None and 0.0 < t - self.kf[k][2] < 0.8 \
                        and self.yaw_ema[k] is not None:
                    x = self.kf[k][0]
                    dtc = t - self.kf[k][2]
                    pub.publish(Float64MultiArray(
                        data=[float(x[0] + x[2] * dtc), float(x[1] + x[3] * dtc),
                              float(x[2]), float(x[3]), float(self.yaw_ema[k]), 1.0]))
                else:
                    pub.publish(Float64MultiArray(data=[0.0] * 5 + [0.0]))
                continue
            rel_w, yaw_w, yaw_ok = out[k]
            # 트랙별 상수속도 KF — 발행 위치·속도는 필터 상태다. 실종 구간에는 전파하지
            # 않고(무효 발행), 재획득 시 실제 경과 dt로 한 스텝 돌려 자연히 이어붙인다.
            dt = t - self.kf[k][2] if self.kf[k] is not None else None
            if dt is None or not (0.0 < dt < 1.0):              # 초기화·긴 공백 -> 재초기화
                x = np.array([rel_w[0], rel_w[1], 0.0, 0.0])
                P = np.diag([self.kf_r ** 2, self.kf_r ** 2, 0.5 ** 2, 0.5 ** 2])
            else:
                x, P = kf_step(self.kf[k][0], self.kf[k][1], rel_w, dt, self.kf_q, self.kf_r)
            self.kf[k] = (x, P, t)
            self.tracks[k] = rel_w                              # 게이트 앵커는 생측정 유지(트래킹 시맨틱 불변)
            # 헤딩 EMA(2026-08-10): 희소 링(셀 3~7개) 라인피팅 yaw는 잡음이 커
            # (S4 실측 중앙값 32~43°) 원측정 대신 각도 EMA(α=0.65)를 발행 — PCA 프레임 잡음 ~2°라 강평활 불필요; α 0.25의 랙(τ~0.4s)이 요동 중 12~20° 오차 주인(S4 실측 15°). 공백
            # 2 s 넘으면 리셋해 오래된 값에 안 끌린다.
            if self.yaw_ema[k] is None or (dt is not None and dt > 2.0):
                self.yaw_ema[k] = yaw_w                 # 부트스트랩·리셋은 게이트 무관(발행 연속성)
            elif yaw_ok:                                # 아스펙트 붕괴 프레임은 EMA 보류(프로브 D)
                # α 0.65→0.5(프로브 G): 랙 우려는 kpsi 0.15로 요동이 줄어(S5 6~7°) 약화 —
                # 게이트 통과 프레임 위 평활 강화로 S4 잔여 잡음(gtbot3 10.2° 경계) 마감.
                e = np.arctan2(np.sin(yaw_w - self.yaw_ema[k]),
                               np.cos(yaw_w - self.yaw_ema[k]))
                self.yaw_ema[k] = float(np.arctan2(
                    np.sin(self.yaw_ema[k] + 0.5 * e),
                    np.cos(self.yaw_ema[k] + 0.5 * e)))
            pub.publish(Float64MultiArray(
                data=[float(x[0]), float(x[1]), float(x[2]), float(x[3]),
                      float(self.yaw_ema[k]), 1.0]))

def main():
    rclpy.init()
    rclpy.spin(PlatformPerception())
