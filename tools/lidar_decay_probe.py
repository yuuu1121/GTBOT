"""LiDAR 발행률의 시간 열화를 구간별로 잰다 — 무엇이 열화를 만드는가.

배경(2026-08-17): `/ouster/points`가 기동 직후엔 선언값 10 Hz를 정확히 내다가
가동 시간에 따라 2.5 Hz로, 국소적으로 0.04 Hz까지 무너진다. 그 사이에도 RTF는
0.997, odometry 9.99 Hz로 물리는 멀쩡하다 — 래스터화 경로만 죽는다. 이 열화가
velocity_loop의 0.5 s 신선도 게이트를 트립시켜 편대를 무너뜨린다(sim-results.md).

sim_perf_probe.py와의 차이: 저건 한 창(單窓)의 평균을 낸다. 열화는 시간 함수라
평균 하나로는 안 보인다 — 이 도구는 60 s 창을 연속으로 찍어 **기울기**를 낸다.

같이 찍는 것:
  RTF·odometry    물리 경로가 같이 죽는지(= 시뮬 전체 감속) 아니면 렌더만인지.
  RSS·GPU 메모리  누수라면 단조 증가가 보인다. 안 보이면 누수 가설이 죽는다.

사용: lidar_decay_probe.py [총초] [창초]
"""
import subprocess
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

TOTAL = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
WIN = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)

cnt = {'lidar': 0, 'odom': 0}
stamp = {'first': None, 'last': None}
rclpy.init()
node = Node('lidar_decay_probe')


def on_odom(m):
    cnt['odom'] += 1
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if stamp['first'] is None:
        stamp['first'] = t
    stamp['last'] = t


node.create_subscription(PointCloud2, '/ouster/points',
                         lambda m: cnt.__setitem__('lidar', cnt['lidar'] + 1), BE)
node.create_subscription(Odometry, '/platform/odometry', on_odom, 10)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()


def sim_mem():
    """stonefish_simulator의 (CPU%, RSS MiB). 누수라면 RSS가 단조 증가한다."""
    try:
        out = subprocess.run(['ps', '-C', 'stonefish_simulator', '-o', '%cpu=,rss='],
                             capture_output=True, text=True, timeout=5).stdout.split()
        return float(out[0]), float(out[1]) / 1024.0
    except Exception:
        return float('nan'), float('nan')


def gpu_mem():
    """GPU 사용 메모리 MiB — 없으면 nan(누수 판정은 RSS로 대체)."""
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=memory.used',
                              '--format=csv,noheader,nounits'],
                             capture_output=True, text=True, timeout=5).stdout
        return float(out.split()[0])
    except Exception:
        return float('nan')


print(f'{TOTAL:.0f} s를 {WIN:.0f} s 창으로 분할 측정', flush=True)
time.sleep(3.0)                          # 구독 성립 대기
print('  경과   LiDAR Hz  odom Hz    RTF   CPU%  RSS MiB  GPU MiB', flush=True)
t_run = time.time()
while time.time() - t_run < TOTAL:
    for k in cnt:
        cnt[k] = 0
    stamp['first'] = stamp['last'] = None
    t0 = time.time()
    time.sleep(WIN)
    w = time.time() - t0
    rtf = ((stamp['last'] - stamp['first']) / w
           if stamp['first'] is not None and stamp['last'] is not None else float('nan'))
    cpu, rss = sim_mem()
    print(f'{time.time()-t_run:6.0f}s  {cnt["lidar"]/w:8.2f}  {cnt["odom"]/w:7.2f}  '
          f'{rtf:5.3f}  {cpu:5.0f}  {rss:7.0f}  {gpu_mem():7.0f}', flush=True)

# spin 스레드가 살아있는 채로 shutdown하면 종료 시 core dump가 난다(실측).
# os._exit는 stdout 버퍼를 비우지 않으므로 반드시 먼저 flush(2026-08-16 실측 교훈).
import os
sys.stdout.flush()
os._exit(0)
