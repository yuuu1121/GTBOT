"""사각 경로 편대주행 탑뷰 녹화 — odometry로 직접 그린다(화면 캡처 없음).

왜 화면 캡처가 아닌가: 종전 `record_square_map.py`는 Stonefish GUI 창을 mss로 긁어
`WIN`/`CROP`이 그때 창 좌표·수동 패닝 상태에 묶여 있었다(카메라가 대상을 픽셀 추적해
패닝을 다시 잡아야 하는 문제도 있다 — 이것으로 녹화 3회를 버렸다). 이 도구는 토픽만
쓰므로 GUI 상태와 무관하게 언제든 같은 결과가 나온다.

무엇이 보이는가:
  회색 사각형 = 리더에게 준 지령 경로,  주황 궤적 = 리더가 실제로 지난 자취
  색 원 = 팔로워 3대(화살표 = 헤딩). 지각이 죽은 로봇은 회색 + LOST
  가는 선 = 편대 변(로봇-로봇). 목표 길이에서 벗어난 만큼 붉어진다
  하단 = 경과·리더 주행거리·변 오차·로봇별 est 유효율

사용: record_square_run.py <초> <출력> [캡처fps] [출력fps]
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
from std_msgs.msg import Float64MultiArray

from gtbot_formation.relative_state import OFFSETS

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 420.0
OUT = sys.argv[2] if len(sys.argv) > 2 else 'square_run.mp4'
CAP_FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
OUT_FPS = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
SIZE = 900
ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']
COLS = [(70, 70, 230), (70, 200, 70), (230, 160, 60)]     # BGR
SQ = [(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)]     # leader_pilot 기본 웨이포인트
LO, HI = -3.5, 11.5                                        # 표시 범위 [m]

# 편대 변의 목표 길이 — OFFSETS(스테이션 배치)에서 유도한다(하드코딩 금지)
PAIRS = [(0, 1), (1, 2), (2, 0)]
TARGET = [math.dist(OFFSETS[i], OFFSETS[j]) for i, j in PAIRS]

pose, yaw, valid = {}, {}, {}
rclpy.init()
node = Node('square_run_video')


def to_px(p):
    u = (p[0] - LO) / (HI - LO) * SIZE
    v = SIZE - (p[1] - LO) / (HI - LO) * SIZE
    return int(u), int(v)


def mk_odom(n):
    def cb(m):
        pose[n] = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        yaw[n] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z))
    return cb


def mk_est(n):
    def cb(m):
        valid[n] = m.data[5] > 0.5
    return cb


node.create_subscription(Odometry, '/platform/odometry', mk_odom('platform'), 10)
for r in ROBOTS:
    node.create_subscription(Odometry, f'/{r}/odometry', mk_odom(r), 10)
    node.create_subscription(Float64MultiArray, f'/{r}/state_est', mk_est(r), 10)
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()


def draw(t, trail, dist, vstat):
    img = np.full((SIZE, SIZE, 3), 24, np.uint8)
    for g in range(-3, 12):                                    # 1 m 격자
        p = to_px((g, g))
        cv2.line(img, (p[0], 0), (p[0], SIZE), (40, 40, 40), 1)
        cv2.line(img, (0, p[1]), (SIZE, p[1]), (40, 40, 40), 1)
    sq = np.array([to_px(c) for c in SQ], np.int32)
    cv2.polylines(img, [sq], True, (150, 150, 150), 2, cv2.LINE_AA)
    for c in SQ:
        cv2.circle(img, to_px(c), 6, (200, 200, 200), -1, cv2.LINE_AA)
    if len(trail) > 1:
        cv2.polylines(img, [np.array([to_px(p) for p in trail], np.int32)],
                      False, (60, 170, 255), 2, cv2.LINE_AA)
    # 편대 변 — 목표 대비 오차만큼 붉게
    errs = []
    for (i, j), tg in zip(PAIRS, TARGET):
        a, b = ROBOTS[i], ROBOTS[j]
        if a not in pose or b not in pose:
            continue
        e = abs(math.dist(pose[a], pose[b]) - tg)
        errs.append(e)
        k = min(e / 0.5, 1.0)
        cv2.line(img, to_px(pose[a]), to_px(pose[b]),
                 (int(200 * (1 - k)), int(200 * (1 - k)), int(90 + 165 * k)),
                 2, cv2.LINE_AA)
    if 'platform' in pose:
        cv2.circle(img, to_px(pose['platform']), 10, (40, 140, 255), -1, cv2.LINE_AA)
        cv2.putText(img, 'platform (leader)', (to_px(pose['platform'])[0] + 14,
                                               to_px(pose['platform'])[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 140, 255), 1, cv2.LINE_AA)
    for r, col in zip(ROBOTS, COLS):
        if r not in pose:
            continue
        ok = valid.get(r, False)
        c = col if ok else (110, 110, 110)
        u, v = to_px(pose[r])
        cv2.circle(img, (u, v), 8, c, -1, cv2.LINE_AA)
        if r in yaw:                                            # 헤딩 화살표
            e = to_px((pose[r][0] + 0.6 * math.cos(yaw[r]),
                       pose[r][1] + 0.6 * math.sin(yaw[r])))
            cv2.arrowedLine(img, (u, v), e, c, 2, cv2.LINE_AA, tipLength=0.35)
        if not ok:
            cv2.putText(img, 'LOST', (u + 12, v - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (80, 80, 240), 2, cv2.LINE_AA)
    med = float(np.median(errs)) if errs else float('nan')
    cv2.putText(img, f't {t:6.1f} s   leader {dist:5.1f} m   edge err {med:.3f} m',
                (14, SIZE - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(img, '  '.join(f'{r} est {100*np.mean(vstat[r]) if vstat[r] else 0:.0f}%'
                               for r in ROBOTS),
                (14, SIZE - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, f'commanded 8x8 m square   ({OUT_FPS/CAP_FPS:.0f}x speed)',
                (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return img, med


vw = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), OUT_FPS, (SIZE, SIZE))
t0 = time.time()
n, dist, last, trail = 0, 0.0, None, []
vstat = {r: [] for r in ROBOTS}
eds = []
while True:
    t = time.time() - t0
    if t >= DUR:
        break
    if 'platform' in pose:
        p = pose['platform']
        if last is not None:
            dist += math.dist(p, last)
        last = p
        if not trail or math.dist(p, trail[-1]) > 0.05:
            trail.append(p)
    for r in ROBOTS:
        vstat[r].append(1.0 if valid.get(r, False) else 0.0)
    f, med = draw(t, trail, dist, vstat)
    if not math.isnan(med):
        eds.append(med)
    vw.write(f)
    n += 1
    if n % 120 == 0:
        print(f'  {t:6.1f}s  leader {dist:5.1f} m  edge {med:.3f} m', flush=True)
    s = (n / CAP_FPS) - (time.time() - t0)
    if s > 0:
        time.sleep(s)
vw.release()
e = np.array(eds) if eds else np.array([float('nan')])
print(f'DONE {n} 프레임 -> {OUT} ({os.path.getsize(OUT)/1e6:.1f} MB, '
      f'{n/OUT_FPS:.0f} s 영상, {OUT_FPS/CAP_FPS:.0f}배속)', flush=True)
print(f'리더 주행 {dist:.1f} m, 변 오차 중앙 {np.nanmedian(e):.3f} m '
      f'p90 {np.nanpercentile(e,90):.3f} m', flush=True)
for r in ROBOTS:
    print(f'  {r} est 유효율 {100*np.mean(vstat[r]):.1f}%', flush=True)
sys.stdout.flush()   # os._exit는 버퍼를 비우지 않는다 — 여기서 직접 비운다
try:
    node.destroy_node()
    rclpy.shutdown()
except Exception:
    pass
os._exit(0)
