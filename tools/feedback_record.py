"""되먹임 시험 기록기 — 교란 주입 + 로봇별 조준이탈·검출유효·스테이션오차 시계열.

타임라인: 0~30 s 기준선 -> 30~38 s gtbot에 공통모드 추력(yaw 스핀) -> 38~200 s 관찰.
공통모드로 주는 이유: 병진이 아니라 **판을 눕히는 것**이 시험 대상이기 때문이다.
velocity_loop도 같은 토픽에 쓰므로 교란 구간에는 50 Hz로 덮어써 우선권을 갖는다.
"""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from gtbot_formation.mixer import wrap, yaw_of
from gtbot_formation.relative_state import OFFSETS

OUT = sys.argv[1] if len(sys.argv) > 1 else 'feedback.csv'
ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']
T_BASE, T_PUSH, T_END = 30.0, 38.0, 200.0
PUSH = 0.35            # 공통모드 setpoint — 요 권위 안에서 판을 확실히 눕히는 크기

rclpy.init()
n = Node('feedback_record')
gt, est = {}, {}


def mk_gt(k):
    def cb(m):
        q = m.pose.pose.orientation
        gt[k] = (np.array([m.pose.pose.position.x, m.pose.pose.position.y]),
                 yaw_of(q.x, q.y, q.z, q.w))
    return cb


def mk_est(k):
    def cb(m):
        est[k] = (np.array(m.data[0:2]), m.data[4], m.data[5])
    return cb


for k in ['platform'] + ROBOTS:
    n.create_subscription(Odometry, f'/{k}/odometry', mk_gt(k), 10)
for k in ROBOTS:
    n.create_subscription(Float64MultiArray, f'/{k}/state_est', mk_est(k), 10)
thr = n.create_publisher(Float64MultiArray, '/gtbot/thrusters', 10)

f = open(OUT, 'w')
cols = ['t', 'push']
for r in ROBOTS:
    cols += [f'{r}_delta_deg', f'{r}_valid', f'{r}_station_err', f'{r}_phi_deg']
f.write(','.join(cols) + '\n')

t0 = time.time()
last = 0.0
while time.time() - t0 < T_END:
    rclpy.spin_once(n, timeout_sec=0.02)
    t = time.time() - t0
    pushing = T_BASE <= t < T_PUSH
    if pushing:
        thr.publish(Float64MultiArray(data=[PUSH] * 4))     # 공통모드 = yaw 스핀
    if t - last < 0.1 or 'platform' not in gt or any(r not in gt for r in ROBOTS):
        continue
    last = t
    pP, _ = gt['platform']
    row = [f'{t:.2f}', '1' if pushing else '0']
    for r, off in zip(ROBOTS, OFFSETS):
        pG, yG = gt[r]
        rel = pG - pP
        bearing = np.arctan2(-rel[1], -rel[0])          # 로봇 -> 플랫폼
        delta = abs(np.degrees(wrap(yG - bearing)))     # 조준 이탈(참값 기준)
        phi = 90.0 - delta                              # 시선-판 각
        v = est[r][2] if r in est else 0.0
        serr = float(np.linalg.norm(rel - np.array(off)))
        row += [f'{delta:.2f}', f'{v:.0f}', f'{serr:.4f}', f'{phi:.2f}']
    f.write(','.join(row) + '\n')
f.close()
print(f'recorded {OUT}')
