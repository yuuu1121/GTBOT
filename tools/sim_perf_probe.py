"""시뮬 성능 병목 계측 — 어느 스테이지가 늦는가.

무엇을 재는가:
  실시간 배속 (RTF)  odometry 헤더 스탬프(시뮬 시계) 진행 / 벽시계 진행.
                     1.0 미만이면 시뮬이 실시간을 못 따라간다 = 이게 '렉'의 정체.
  토픽별 실효 Hz     선언 rate와 비교해 어느 센서가 밀리는지 본다.
                     /ouster/points(LiDAR 10 Hz 선언, 스캔당 131k 레이),
                     /gtbot_world/view/image_color(카메라 10 Hz 선언),
                     /platform/odometry(10 Hz, 물리 스텝에 붙음 — 물리 부하의 대리).
  프로세스 CPU        stonefish_simulator의 CPU 점유(코어 환산).

RTF와 odometry Hz가 같이 낮으면 물리·시뮬 루프가 병목이고, 그 둘은 정상인데
LiDAR·카메라만 낮으면 렌더가 병목이다. 이 구분이 메시 감량의 값어치를 정한다.

사용: sim_perf_probe.py [측정초]
"""
import subprocess
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)

cnt = {'lidar': 0, 'cam': 0, 'odom': 0}
stamp = {'first': None, 'last': None}
rclpy.init()
node = Node('sim_perf_probe')


def on_odom(m):
    cnt['odom'] += 1
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if stamp['first'] is None:
        stamp['first'] = t
    stamp['last'] = t


node.create_subscription(PointCloud2, '/ouster/points',
                         lambda m: cnt.__setitem__('lidar', cnt['lidar'] + 1), BE)
node.create_subscription(Image, '/gtbot_world/view/image_color',
                         lambda m: cnt.__setitem__('cam', cnt['cam'] + 1), BE)
node.create_subscription(Odometry, '/platform/odometry', on_odom, 10)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()


def sim_cpu():
    """stonefish_simulator의 CPU 점유(%) — 100%가 코어 1개."""
    try:
        out = subprocess.run(['ps', '-C', 'stonefish_simulator', '-o', '%cpu='],
                             capture_output=True, text=True, timeout=5).stdout
        return sum(float(x) for x in out.split()) if out.strip() else float('nan')
    except Exception:
        return float('nan')


print(f'{DUR:.0f} s 측정 시작', flush=True)
time.sleep(3.0)                      # 구독 성립 대기
for k in cnt:
    cnt[k] = 0
stamp['first'] = stamp['last'] = None
t0 = time.time()
cpu = []
while time.time() - t0 < DUR:
    time.sleep(2.0)
    cpu.append(sim_cpu())
wall = time.time() - t0

print(f'\n벽시계 {wall:.1f} s')
if stamp['first'] is not None and stamp['last'] is not None:
    sim_dt = stamp['last'] - stamp['first']
    print(f'  시뮬 시계 진행 {sim_dt:.1f} s   -> 실시간 배속(RTF) {sim_dt/wall:.3f}')
else:
    print('  odometry 스탬프 없음 — RTF 계산 불가')
for k, decl in [('lidar', 10.0), ('cam', 10.0), ('odom', 10.0)]:
    hz = cnt[k] / wall
    print(f'  {k:<6} {cnt[k]:5d}건  실효 {hz:5.2f} Hz  (선언 {decl:.0f} Hz, '
          f'달성률 {100*hz/decl:.0f}%)')
c = [x for x in cpu if x == x]
if c:
    print(f'  stonefish CPU 중앙 {sorted(c)[len(c)//2]:.0f}% (코어 {sorted(c)[len(c)//2]/100:.1f}개)')
# spin 스레드가 살아있는 채로 shutdown하면 종료 시 core dump가 난다(실측).
# 다만 os._exit는 stdout 버퍼를 비우지 않으므로 반드시 먼저 flush해야 한다 —
# 이걸 빠뜨려 측정 결과가 통째로 사라진 적이 있다(2026-08-16).
import os
sys.stdout.flush()
os._exit(0)
