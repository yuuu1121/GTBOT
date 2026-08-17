"""시뮬 판 클러스터를 실물 bag과 같은 지표로 재서 나란히 찍는다.

왜 필요한가(2026-08-17): 시뮬 LiDAR를 실물 OS0에 맞추는 작업의 합격/불합격 판정이
매번 같은 잣대여야 한다. 실물 bag(results/real_lidar_data, 검출 후 box_points)에서
뽑은 목표값과 **같은 계산식**으로 시뮬 점군을 재야 비교가 성립한다.

목표값(실물 3 bag 실측):
  발행률 10.00 Hz / 점수 134~157 / 장축 15.4~15.9 cm / 두께 1.3~2.0 cm
  종횡비 0.085~0.129 / 강도 중앙 19.6~21.2 (거리 1.03~1.07 m)

판 선별: 강도 임계 이상(ouster_cluster의 bev_refl_min 5.0과 같은 뜻)인 점을 모아
근접 클러스터로 나눈다. 실물 box_points가 '검출기가 판이라고 판정한 점'이므로
시뮬에서도 같은 성격의 집합을 만들어야 한다.

두께·장축은 주성분 표준편차의 4배(±2σ)로 정의한다 — 실물 분석과 동일 정의.

사용: sim_plate_stats.py [측정초] [강도임계]
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
IMIN = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)

frames = []          # (t, [클러스터별 지표])
t_stamp = []


def cluster(xy, link=0.3, min_pts=10):
    """실물 검출기와 같은 뜻의 근접 연결 클러스터링(perception_core.cluster_2d 규약)."""
    out, used = [], np.zeros(len(xy), bool)
    for i in range(len(xy)):
        if used[i]:
            continue
        idx = [i]; used[i] = True; q = [i]
        while q:
            j = q.pop()
            d = np.linalg.norm(xy - xy[j], axis=1)
            new = np.where((d < link) & (~used))[0]
            used[new] = True
            idx.extend(new.tolist()); q.extend(new.tolist())
        if len(idx) >= min_pts:
            out.append(np.array(idx))
    return out


def on_pc(m):
    t_stamp.append(time.time())
    p = np.array([[a, b, c, d] for a, b, c, d in pc2.read_points(
        m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)])
    if len(p) == 0:
        return
    sel = p[p[:, 3] >= IMIN]
    if len(sel) < 10:
        frames.append([])
        return
    rows = []
    for idx in cluster(sel[:, :2]):
        c = sel[idx]
        xy = c[:, :2] - c[:, :2].mean(0)
        w = np.linalg.svd(xy, compute_uv=False) / math.sqrt(len(xy))
        rows.append((len(c), 4 * w[0], 4 * w[1],
                     np.linalg.norm(c[:, :2].mean(0)), np.median(c[:, 3])))
    frames.append(rows)


rclpy.init()
node = Node('sim_plate_stats')
node.create_subscription(PointCloud2, '/ouster/points', on_pc, BE)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
print(f'{DUR:.0f} s 측정 (강도 임계 {IMIN})', flush=True)
time.sleep(DUR)

hz = (len(t_stamp) - 1) / (t_stamp[-1] - t_stamp[0]) if len(t_stamp) > 1 else 0.0
rows = [r for f in frames for r in f]
print(f'\n{"지표":<10} {"시뮬":>12} {"실물 목표":>16}  판정')
def line(name, val, lo, hi, fmt='{:.2f}'):
    ok = '통과' if val == val and lo <= val <= hi else '미달'
    print(f'{name:<10} {fmt.format(val):>12} {fmt.format(lo)+"~"+fmt.format(hi):>16}  {ok}')

line('발행률Hz', hz, 9.5, 10.5)
if not rows:
    print('\n판 클러스터 0개 — 강도 임계 아래이거나 로봇이 시야 밖이다.')
else:
    a = np.array(rows)
    # 점수와 강도는 1/r² 지표라 거리를 맞추지 않으면 비교가 성립하지 않는다.
    # 시뮬 배치가 실물 촬영 거리(1.05 m)와 달라 실물 기준 거리로 환산해 판정한다.
    r_real, r_sim = 1.05, float(np.median(a[:, 3]))
    k = (r_sim / r_real) ** 2
    line('점수(환산)', float(np.median(a[:, 0])) * k, 134, 157, '{:.0f}')
    line('장축cm', float(np.median(a[:, 1])) * 100, 15.4, 15.9)
    line('두께cm', float(np.median(a[:, 2])) * 100, 1.3, 2.0)
    line('종횡비', float(np.median(a[:, 2] / a[:, 1])), 0.085, 0.129, '{:.3f}')
    line('강도중앙(환산)', float(np.median(a[:, 4])) * k, 19.6, 21.2)
    print(f'\n(참고) 클러스터 {len(rows)}개, 거리 중앙 {r_sim:.2f} m (실물 {r_real:.2f} m)'
          f' — 점수·강도는 x{k:.2f}로 환산해 판정했다. 원값 점수 '
          f'{np.median(a[:, 0]):.0f}, 강도 {np.median(a[:, 4]):.2f}.')

# spin 스레드가 살아있는 채로 shutdown하면 core dump — os._exit 전 flush 필수.
import os
sys.stdout.flush()
os._exit(0)
