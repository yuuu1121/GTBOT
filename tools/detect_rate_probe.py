"""검출 경로의 로봇별 유효율을 잰다 — 실기 경로 전환의 합격 판정.

왜 필요한가(2026-08-17): platform_perception을 input_mode='boxes'로 바꿔 실물 검출기
(ouster_cluster)를 태우는 전환이 실제로 되는지 보려면 '검출이 얼마나 살아있나'를
로봇별로 봐야 한다. state_est는 새 검출이 없어도 고정 주기로 발행되고 valid 플래그로
구분하므로(velocity_loop.on_est와 같은 규약) **발행률이 아니라 valid 비율**이 지표다.

같이 찍는 것:
  /ouster_cluster/boxes  실물 검출기가 판을 몇 개 내는가(단계별 어디서 죽는지 가늠).
  /ouster/points         원 점군 발행률(검출률과 분리해서 봐야 한다).

과거 이 경로는 검출률 ~5%로 BLOCKED였다(sim-results.md OS0 캠페인) — 원인은
'라인 스테이지에서 세그먼트 0'과 판 두께 게이트 탈락이었다. 두께를 실물에 맞춘 뒤
그 판정이 유지되는지가 이 도구로 갈린다.

사용: detect_rate_probe.py [측정초]
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)
ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

est = {r: [] for r in ROBOTS}     # valid 플래그 이력
cnt = {'lidar': 0, 'boxes': 0, 'box_markers': 0}

rclpy.init()
node = Node('detect_rate_probe')


def mk_est(r):
    def cb(m):
        # state_est 레이아웃: [..., valid] — velocity_loop.on_est와 같은 인덱스(5)
        est[r].append(1.0 if len(m.data) > 5 and bool(m.data[5]) else 0.0)
    return cb


def on_boxes(m):
    cnt['boxes'] += 1
    cnt['box_markers'] += len(m.markers)


for r in ROBOTS:
    node.create_subscription(Float64MultiArray, f'/{r}/state_est', mk_est(r), 10)
node.create_subscription(MarkerArray, '/ouster_cluster/boxes', on_boxes, 5)
node.create_subscription(PointCloud2, '/ouster/points',
                         lambda m: cnt.__setitem__('lidar', cnt['lidar'] + 1), BE)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

print(f'{DUR:.0f} s 측정', flush=True)
time.sleep(2.0)
for r in ROBOTS:
    est[r].clear()
for k in cnt:
    cnt[k] = 0
t0 = time.time()
time.sleep(DUR)
w = time.time() - t0

print(f'\n벽시계 {w:.1f} s')
print(f'  /ouster/points          {cnt["lidar"]:5d}건  {cnt["lidar"]/w:5.2f} Hz')
print(f'  /ouster_cluster/boxes   {cnt["boxes"]:5d}건  {cnt["boxes"]/w:5.2f} Hz  '
      f'마커 평균 {cnt["box_markers"]/max(cnt["boxes"],1):.2f}개/프레임')
print('\n로봇별 est 유효율 (S4/S6 게이트의 그 지표)')
for r in ROBOTS:
    v = est[r]
    if not v:
        print(f'  {r:8} state_est 없음')
    else:
        print(f'  {r:8} {100*sum(v)/len(v):5.1f}%  ({len(v)}표본, {len(v)/w:5.2f} Hz 발행)')

# spin 스레드가 살아있는 채로 shutdown하면 core dump — os._exit 전 flush 필수.
import os
sys.stdout.flush()
os._exit(0)
