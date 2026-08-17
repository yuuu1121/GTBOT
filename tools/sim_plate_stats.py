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

사용: sim_plate_stats.py [측정초] [강도임계] [분석간격프레임]
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
SKIP = int(sys.argv[3]) if len(sys.argv) > 3 else 10   # 판 분석은 이 간격마다 (발행률 계수는 매 프레임)
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
    # 도착 시각은 **항상** 기록하고(발행률), 무거운 판 분석은 SKIP 프레임마다 한 번만
    # 한다. 종전엔 콜백에서 13만 점을 파이썬으로 다 돌아 콜백이 1 s 넘게 걸렸고,
    # 그 결과 여기 찍히는 '발행률'이 발행 속도가 아니라 **이 도구의 처리 속도**였다
    # (0.33~0.66 Hz로 나왔지만 계수만 하는 lidar_decay_probe는 같은 조건에서 ~5 Hz).
    t_stamp.append(time.time())
    if (len(t_stamp) - 1) % SKIP != 0:
        return
    # 구조화 배열 그대로 받아 벡터로 변환한다. 종전의 점별 파이썬 루프는 13만 점에서
    # 초 단위로 걸려 콜백이 발행 주기를 넘겼다.
    s = pc2.read_points(m, field_names=('x', 'y', 'z', 'intensity'),
                        skip_nans=True, reshape_organized_cloud=False)
    if len(s) == 0:
        return
    p = np.column_stack([np.asarray(s['x']), np.asarray(s['y']),
                         np.asarray(s['z']), np.asarray(s['intensity'])])
    sel = p[p[:, 3] >= IMIN]
    if len(sel) < 10:
        frames.append([])
        return
    rows = []
    for idx in cluster(sel[:, :2]):
        c = sel[idx]
        ctr = c[:, :2].mean(0)
        xy = c[:, :2] - ctr
        u, w, vt = np.linalg.svd(xy, full_matrices=False)
        w = w / math.sqrt(len(xy))
        # 입사각: 판 법선(BEV 단축 방향)과 시선(원점->판 중심)의 각. 점수·강도는
        # cos에 비례하므로 이걸 모르면 '이득이 낮은 것'과 '판이 돌아간 것'을 못 가른다.
        # 실물 bag은 판을 정면으로 향한 조건(φ≈85°, cos≈1)이라 시뮬도 같은 조건만
        # 골라야 비교가 성립한다 — 제어를 안 띄운 시뮬은 로봇이 아무 방향이나 본다.
        n = vt[1] / (np.linalg.norm(vt[1]) + 1e-12)
        v = ctr / (np.linalg.norm(ctr) + 1e-12)
        rows.append((len(c), 4 * w[0], 4 * w[1],
                     np.linalg.norm(ctr), np.median(c[:, 3]), abs(float(n @ v))))
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

line("발행률Hz", hz, 9.5, 10.5)
if not rows:
    print('\n판 클러스터 0개 — 강도 임계 아래이거나 로봇이 시야 밖이다.')
else:
    a0 = np.array(rows)
    # 실물 촬영 조건(판 정면, cos>=0.9)에 해당하는 클러스터만 남긴다. 없으면 전체를
    # 쓰되 그 사실을 알린다 — 조건이 다른 표본으로 낸 판정은 근거가 못 된다.
    a = a0[a0[:, 5] >= 0.9]
    if len(a) < 3:
        print(f'\n주의: 정면(cos>=0.9) 클러스터가 {len(a)}개뿐이라 전체 {len(a0)}개로 판정한다 '
              f'— cos 중앙 {np.median(a0[:, 5]):.2f}. 제어를 띄우지 않아 판이 돌아간 상태다.')
        a = a0
    else:
        print(f'\n정면 조건(cos>=0.9) 클러스터 {len(a)}/{len(a0)}개로 판정 '
              f'(전체 cos 중앙 {np.median(a0[:, 5]):.2f}).')
    # 점수와 강도는 1/r² 지표라 거리를 맞추지 않으면 비교가 성립하지 않는다.
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
