"""LiDAR 판 검출·클러스터링 BEV 영상.

무엇을 보여주는가: 플랫폼 LiDAR가 매 스캔 뱉는 점군을 위에서 내려다본 그림에
  회색 = 강도 낮음(선체 등), 색 = 강도 255(반사판),
  색 점 위의 직선 = 클러스터에 적합한 장축(=추정 헤딩), 점선 = 플랫폼→로봇 시선
을 얹는다. 클러스터 색은 로봇 신원(state_est 위치에 최근접)으로 고정해 프레임 간
색이 튀지 않게 한다.

좌표계는 LiDAR FLU 원본 그대로다(월드 변환 전) — 검출 단계를 보이는 것이 목적이라
하류 변환을 섞지 않는다.

입사각 표시(2026-08-14 추가): 실물 bag에서 뽑은 입사각 의존 검출 곡선을 시뮬에 걸었을
때 무슨 일이 벌어지는지 보이기 위해, 클러스터마다 입사각 φ(90° = 판이 플랫폼을 정면으로
봄)와 그 φ에서의 검출 생존확률을 같이 쓴다. 색이 살아 있으면 그 로봇의 state_est가
유효한 것이고, 회색 + LOST면 지각이 죽은 것이다.

φ는 `/platform/det_phi`로 받아 쓴다 — 검출 생존을 실제로 결정한 그 값이다. 이 도구가
점군에서 직접 재는 φ도 계산은 하지만(짝이 없을 때의 폴백, CSV 기록용) 화면에는 쓰지
않는다: 클러스터링·장축 적합이 검출기와 달라 정면 부근에서 8.5° 낮게 읽는 편의가
실측됐다(상관 0.962, tools/compare_phi.py). 시각화가 파이프라인을 대변해야 하므로
파이프라인 값이 정본이다.

사용: record_plate_cluster.py <초> <출력> [phi_full] [phi_zero]
  phi_zero 기본 65.4 = 실물 이동2 bag에 맞춘 값. 80/50을 주면 순한 가정 곡선.
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.cluster.hierarchy import fcluster, linkage
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else 'plate_cluster.mp4'
SIZE, HALF = 860, 2.2            # 캔버스 px, 표시 반경 m
PPM = SIZE / (2 * HALF)
FPS_OUT = 20
ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']
COLS = [(70, 70, 230), (70, 200, 70), (230, 160, 60)]   # BGR
# 입사각 곡선 — platform_perception의 inc_phi_full/inc_phi_zero와 같은 의미
PHI_FULL = float(sys.argv[3]) if len(sys.argv) > 3 else 80.0
PHI_ZERO = float(sys.argv[4]) if len(sys.argv) > 4 else 65.4


# 5번째 인자를 주면 이 도구가 잰 φ를 CSV로 남긴다(platform_perception의 phi_log와 대조용)
PHI_CSV = open(sys.argv[5], 'w') if len(sys.argv) > 5 else None
if PHI_CSV:
    PHI_CSV.write('t,cx,cy,phi\n')


def keep_prob(phi):
    """이 입사각에서 검출이 살아남을 확률. φ >= phi_full이면 1, phi_zero 이하면 0."""
    return 1.0 - min(max((PHI_FULL - phi) / (PHI_FULL - PHI_ZERO), 0.0), 1.0)


def to_px(x, y):
    """LiDAR FLU(x 전방, y 좌현) -> 화면. 화면은 x 위, y 왼쪽이 되도록 돌린다."""
    return int(SIZE / 2 - y * PPM), int(SIZE / 2 - x * PPM)


est = {}
est_last = {}
rclpy.init()
node = Node('cluster_video')


def mk(r):
    def cb(m):
        est[r] = (np.array(m.data[0:2]), m.data[4], m.data[5])
        if m.data[5] > 0.5:
            # 지각이 죽어도 신원은 붙여야 'LOST'를 그릴 수 있다 — 마지막 유효 위치 보관
            est_last[r] = np.array(m.data[0:2])
    return cb


for r in ROBOTS:
    node.create_subscription(Float64MultiArray, f'/{r}/state_est', mk(r), 10)

# 파이프라인이 실제로 쓴 φ. [yaw_p, (ox, oy, phi, kept) × N] — origin은 월드축 정렬이라
# 화면의 FLU 좌표와 짝지으려면 yaw_p로 R(+yaw_p)·(x,−y)를 되돌려야 한다.
det_phi = {'yaw': 0.0, 'rows': []}


def on_phi(m):
    d = list(m.data)
    det_phi['yaw'] = d[0] if d else 0.0
    det_phi['rows'] = [d[i:i + 4] for i in range(1, len(d) - 3, 4)]


node.create_subscription(Float64MultiArray, '/platform/det_phi', on_phi, 10)


def phi_from_pipeline(c):
    """FLU 중심 c에 해당하는 검출의 파이프라인 φ. 짝이 없으면 None."""
    ca, sa = math.cos(det_phi['yaw']), math.sin(det_phi['yaw'])
    wx = ca * c[0] - sa * (-c[1])
    wy = sa * c[0] + ca * (-c[1])
    best = None
    for ox, oy, ph, kept in det_phi['rows']:
        e = math.hypot(wx - ox, wy - oy)
        if e < 0.15 and (best is None or e < best[0]):
            best = (e, ph, kept)
    return None if best is None else (best[1], best[2])

frames = []
t0 = time.time()
stats = {'n': 0, 'plate_pts': [], 'clusters': [], 'phi': [],
         'valid': {r: [] for r in ROBOTS}}


def draw(msg):
    img = np.full((SIZE, SIZE, 3), 22, np.uint8)
    # 격자 1 m
    for g in range(-2, 3):
        p = int(SIZE / 2 - g * PPM)
        cv2.line(img, (0, p), (SIZE, p), (45, 45, 45), 1)
        cv2.line(img, (p, 0), (p, SIZE), (45, 45, 45), 1)
    cv2.circle(img, (SIZE // 2, SIZE // 2), 9, (40, 140, 255), -1)      # 플랫폼(LiDAR 원점)
    cv2.putText(img, 'platform LiDAR', (SIZE // 2 + 14, SIZE // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 140, 255), 1, cv2.LINE_AA)

    a = np.array([[p[0], p[1], p[3]] for p in
                  pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                  skip_nans=True)])
    if len(a) == 0:
        return img
    lo = a[a[:, 2] <= 200][:, :2]
    hi = a[a[:, 2] > 200][:, :2]
    for x, y in lo:
        cv2.circle(img, to_px(x, y), 1, (105, 105, 105), -1)

    ncl = 0
    if len(hi) >= 6:
        lab = fcluster(linkage(hi, 'single'), 0.25, 'distance')
        for k in np.unique(lab):
            q = hi[lab == k]
            if len(q) < 6:
                continue
            c = q.mean(0)
            # 신원 = 마지막 유효 state_est 위치에 최근접. 지각이 죽어도 붙여 둬야
            # 'LOST'를 그릴 수 있으므로 유효 여부는 색으로만 구분한다.
            cand = [(np.linalg.norm(est_last[r] - c), i) for i, r in enumerate(ROBOTS)
                    if r in est_last]
            hit = min(cand) if cand and min(cand)[0] < 0.6 else None
            alive = hit is not None and est.get(ROBOTS[hit[1]], (0, 0, 0))[2] > 0.5
            col = COLS[hit[1]] if hit and alive else (110, 110, 110)
            for x, y in q:
                cv2.circle(img, to_px(x, y), 2, col, -1)
            d = q - c
            _, s, vt = np.linalg.svd(d, full_matrices=False)
            ext = np.ptp(d @ vt[0]) / 2
            p1, p2 = c + vt[0] * ext, c - vt[0] * ext
            cv2.line(img, to_px(*p1), to_px(*p2), (255, 255, 255), 2, cv2.LINE_AA)
            cv2.line(img, (SIZE // 2, SIZE // 2), to_px(*c), (70, 70, 70), 1, cv2.LINE_AA)
            # 입사각 φ: 판 장축과 시선의 각도차. 장축이 시선에 수직(=판이 정면)이면 90°.
            # 장축은 180° 모호하므로 ±90°로 접는다.
            dpsi = math.atan2(vt[0][1], vt[0][0]) - math.atan2(c[1], c[0])
            phi_own = abs(math.degrees(math.atan2(math.sin(dpsi), math.cos(dpsi))))
            phi_own = 180.0 - phi_own if phi_own > 90.0 else phi_own
            # 파이프라인 값을 우선 쓴다. 자체 측정은 클러스터링·장축 적합이 달라
            # 정면 부근에서 8.5° 낮게 읽는 편의가 실측됐다(tools/compare_phi.py).
            hitp = phi_from_pipeline(c)
            phi = hitp[0] if hitp else phi_own
            pk = keep_prob(phi)
            u, v = to_px(*c)
            cv2.putText(img, f'{len(q)}pt  {2*ext*100:.0f}cm', (u + 12, v - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
            pcol = (90, 220, 90) if pk > 0.99 else (60, 200, 255) if pk > 0 else (80, 80, 240)
            cv2.putText(img, f'phi {phi:4.1f}  keep {pk*100:3.0f}%', (u + 12, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, pcol, 1, cv2.LINE_AA)
            if hit and not alive:
                cv2.putText(img, 'LOST', (u + 12, v + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 240), 2, cv2.LINE_AA)
            stats['phi'].append(phi)
            if PHI_CSV:
                PHI_CSV.write(f'{time.time():.3f},{c[0]:.4f},{c[1]:.4f},{phi_own:.2f}\n')
            ncl += 1
            stats['plate_pts'].append(len(q))
    stats['clusters'].append(ncl)
    cv2.putText(img, f'plate points {len(hi):4d}   clusters {ncl}   '
                     f'hull/other {len(lo):5d}', (14, SIZE - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, 'gray = low intensity / perception dead   colored = tracked plate   '
                     'phi from perception pipeline (90 = plate faces platform)', (14, SIZE - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, f't = {time.time() - t0:5.1f} s', (SIZE - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, f'incidence model  full {PHI_FULL:.0f} deg -> keep 100%   '
                     f'zero {PHI_ZERO:.1f} deg -> keep 0%   (real bag fit)', (14, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 255), 1, cv2.LINE_AA)
    for i, r in enumerate(ROBOTS):
        ok = r in est and est[r][2] > 0.5
        stats['valid'][r].append(1.0 if ok else 0.0)
        cv2.putText(img, f'{r} {"OK  " if ok else "LOST"}', (14, 56 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    COLS[i] if ok else (80, 80, 240), 1, cv2.LINE_AA)
    return img


def on_cloud(msg):
    if time.time() - t0 > DUR:
        return
    frames.append(draw(msg))
    stats['n'] += 1


node.create_subscription(PointCloud2, '/ouster_cluster/points_filtered', on_cloud,
                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST))
while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)

vw = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), FPS_OUT, (SIZE, SIZE))
for f in frames:
    vw.write(f)
vw.release()
if PHI_CSV:
    PHI_CSV.close()
pp = np.array(stats['plate_pts']) if stats['plate_pts'] else np.array([0])
print(f'{len(frames)} 프레임 -> {OUT} ({len(frames)/FPS_OUT:.0f} s @ {FPS_OUT} fps)')
print(f'클러스터 수 중앙 {np.median(stats["clusters"]):.0f}, '
      f'클러스터당 점 수 중앙 {np.median(pp):.0f}')
ph = np.array(stats['phi']) if stats['phi'] else np.array([np.nan])
print(f'입사각 φ 중앙 {np.nanmedian(ph):.1f}°, 최저 {np.nanmin(ph):.1f}°, '
      f'생존확률 1 미만인 프레임 {100*np.mean(ph < PHI_FULL):.1f}%')
for r in ROBOTS:
    v = stats['valid'][r]
    print(f'  {r} est 유효율 {100*np.mean(v) if v else 0:.1f}%')
