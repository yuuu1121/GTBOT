"""스톤피시 관측 카메라(`/gtbot_world/view/image_color`) 녹화 — 시뮬 3D 화면 그대로.

왜 이 경로인가: GUI 창을 mss로 긁는 종전 방식(record_square_map.py)은 창 좌표와
수동 패닝 상태에 묶여 재현이 안 된다(그 문제로 녹화 3회를 버렸다). scn의 고정 관측
카메라는 토픽으로 프레임을 내므로 GUI와 무관하게 매번 같은 화면이 나온다.

화면 위에 최소한의 HUD만 얹는다 — 경과, 리더 주행거리, 편대 변 오차, 로봇별 est 유효율.
경로·궤적을 그리는 탑뷰 도식이 필요하면 record_square_run.py 쪽을 쓴다.

기록은 **프레임 도착 이벤트**로 한다(타이머 샘플링 금지) — 카메라 실효율이
선언값 10 Hz가 아니라 1.9 Hz라 타이머로 뽑으면 37%가 중복이 되어 뚝뚝 끊긴다.

사용: record_sim_view.py <초> <출력> [미사용] [출력fps]
"""
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from gtbot_formation.relative_state import OFFSETS

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 660.0
OUT = sys.argv[2] if len(sys.argv) > 2 else 'sim_view.mp4'
CAP_FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
OUT_FPS = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']
PAIRS = [(0, 1), (1, 2), (2, 0)]
TARGET = [math.dist(OFFSETS[i], OFFSETS[j]) for i, j in PAIRS]

pose, valid = {}, {}
frame = {'img': None, 'seq': 0}
rclpy.init()
node = Node('sim_view_video')


def mk_odom(n):
    def cb(m):
        pose[n] = (m.pose.pose.position.x, m.pose.pose.position.y)
    return cb


def mk_est(n):
    def cb(m):
        valid[n] = m.data[5] > 0.5
    return cb


def on_img(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    # stonefish 카메라는 rgb8 — OpenCV는 BGR이라 뒤집는다. 채널이 4면 알파를 버린다.
    a = a[:, :, :3]
    frame['img'] = np.ascontiguousarray(a[:, :, ::-1] if m.encoding == 'rgb8' else a)
    frame['seq'] += 1          # 새 프레임 도착 표시 — 기록 루프가 이걸 보고 쓴다


node.create_subscription(Odometry, '/platform/odometry', mk_odom('platform'), 10)
for r in ROBOTS:
    node.create_subscription(Odometry, f'/{r}/odometry', mk_odom(r), 10)
    node.create_subscription(Float64MultiArray, f'/{r}/state_est', mk_est(r), 10)
node.create_subscription(Image, '/gtbot_world/view/image_color', on_img,
                         QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST))
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

print('첫 프레임 대기...', flush=True)
t_wait = time.time()
while frame['img'] is None and time.time() - t_wait < 60.0:
    time.sleep(0.2)
if frame['img'] is None:
    print('카메라 프레임이 안 온다 — /gtbot_world/view/image_color 확인 필요', flush=True)
    os._exit(2)
H, W = frame['img'].shape[:2]
print(f'프레임 {W}x{H} 수신', flush=True)

vw = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), OUT_FPS, (W, H))
t0 = time.time()
n, dist, last, last_seq = 0, 0.0, None, -1
vstat = {r: [] for r in ROBOTS}
eds = []
while True:
    t = time.time() - t0
    if t >= DUR:
        break
    if frame['seq'] == last_seq:          # 새 프레임 없으면 기다린다(중복 기록 금지)
        time.sleep(0.01)
        continue
    last_seq = frame['seq']
    img = frame['img'].copy()
    if 'platform' in pose:
        p = pose['platform']
        if last is not None:
            dist += math.dist(p, last)
        last = p
    errs = []
    for (i, j), tg in zip(PAIRS, TARGET):
        a, b = ROBOTS[i], ROBOTS[j]
        if a in pose and b in pose:
            errs.append(abs(math.dist(pose[a], pose[b]) - tg))
    med = float(np.median(errs)) if errs else float('nan')
    if not math.isnan(med):
        eds.append(med)
    for r in ROBOTS:
        vstat[r].append(1.0 if valid.get(r, False) else 0.0)
    cv2.rectangle(img, (0, H - 62), (W, H), (0, 0, 0), -1)
    spd = OUT_FPS / max(n / max(t, 1e-6), 0.1)
    cv2.putText(img, f't {t:6.1f} s   leader {dist:5.1f} m   edge err {med:.3f} m'
                     f'   ({spd:.0f}x)', (14, H - 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(img, '   '.join(f'{r} {"OK" if valid.get(r) else "LOST"} '
                                f'{100*np.mean(vstat[r]):.0f}%' for r in ROBOTS),
                (14, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
    vw.write(img)
    n += 1
    if n % 120 == 0:
        print(f'  {t:6.1f}s  leader {dist:5.1f} m  edge {med:.3f} m  '
              f'실효 {n/max(t,1e-6):.1f} Hz', flush=True)
vw.release()
e = np.array(eds) if eds else np.array([float('nan')])
print(f'DONE {n} 프레임 -> {OUT} ({os.path.getsize(OUT)/1e6:.1f} MB, '
      f'{n/OUT_FPS:.0f} s 영상, 실효 캡처 {n/max(DUR,1e-6):.2f} Hz)', flush=True)
print(f'리더 주행 {dist:.1f} m, 변 오차 중앙 {np.nanmedian(e):.3f} m '
      f'p90 {np.nanpercentile(e,90):.3f} m', flush=True)
for r in ROBOTS:
    print(f'  {r} est 유효율 {100*np.mean(vstat[r]):.1f}%', flush=True)
sys.stdout.flush()
try:
    node.destroy_node()
    rclpy.shutdown()
except Exception:
    pass
os._exit(0)
