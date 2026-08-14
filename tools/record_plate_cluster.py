"""LiDAR 판 검출·클러스터링 BEV 영상.

무엇을 보여주는가: 플랫폼 LiDAR가 매 스캔 뱉는 점군을 위에서 내려다본 그림에
  회색 = 강도 낮음(선체 등), 색 = 강도 255(반사판),
  색 점 위의 직선 = 클러스터에 적합한 장축(=추정 헤딩), 점선 = 플랫폼→로봇 시선
을 얹는다. 클러스터 색은 로봇 신원(state_est 위치에 최근접)으로 고정해 프레임 간
색이 튀지 않게 한다.

좌표계는 LiDAR FLU 원본 그대로다(월드 변환 전) — 검출 단계를 보이는 것이 목적이라
하류 변환을 섞지 않는다.
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


def to_px(x, y):
    """LiDAR FLU(x 전방, y 좌현) -> 화면. 화면은 x 위, y 왼쪽이 되도록 돌린다."""
    return int(SIZE / 2 - y * PPM), int(SIZE / 2 - x * PPM)


est = {}
rclpy.init()
node = Node('cluster_video')


def mk(r):
    def cb(m):
        est[r] = (np.array(m.data[0:2]), m.data[4], m.data[5])
    return cb


for r in ROBOTS:
    node.create_subscription(Float64MultiArray, f'/{r}/state_est', mk(r), 10)

frames = []
t0 = time.time()
stats = {'n': 0, 'plate_pts': [], 'clusters': []}


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
            # 신원 = state_est 위치에 최근접(유효한 것만). 없으면 회색으로 그린다.
            cand = [(np.linalg.norm(est[r][0] - c), i) for i, r in enumerate(ROBOTS)
                    if r in est and est[r][2] > 0.5]
            col = COLS[min(cand)[1]] if cand and min(cand)[0] < 0.6 else (200, 200, 200)
            for x, y in q:
                cv2.circle(img, to_px(x, y), 2, col, -1)
            d = q - c
            _, s, vt = np.linalg.svd(d, full_matrices=False)
            ext = np.ptp(d @ vt[0]) / 2
            p1, p2 = c + vt[0] * ext, c - vt[0] * ext
            cv2.line(img, to_px(*p1), to_px(*p2), (255, 255, 255), 2, cv2.LINE_AA)
            cv2.line(img, (SIZE // 2, SIZE // 2), to_px(*c), (70, 70, 70), 1, cv2.LINE_AA)
            u, v = to_px(*c)
            cv2.putText(img, f'{len(q)}pt  {2*ext*100:.0f}cm', (u + 12, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
            ncl += 1
            stats['plate_pts'].append(len(q))
    stats['clusters'].append(ncl)
    cv2.putText(img, f'plate points {len(hi):4d}   clusters {ncl}   '
                     f'hull/other {len(lo):5d}', (14, SIZE - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, 'gray = low intensity   colored = retroreflective plate (I=255)   '
                     'white line = fitted major axis', (14, SIZE - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, f't = {time.time() - t0:5.1f} s', (SIZE - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
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
pp = np.array(stats['plate_pts']) if stats['plate_pts'] else np.array([0])
print(f'{len(frames)} 프레임 -> {OUT} ({len(frames)/FPS_OUT:.0f} s @ {FPS_OUT} fps)')
print(f'클러스터 수 중앙 {np.median(stats["clusters"]):.0f}, '
      f'클러스터당 점 수 중앙 {np.median(pp):.0f}')
