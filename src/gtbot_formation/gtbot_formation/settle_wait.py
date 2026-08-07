"""정착 대기 — 워밍업 종료(제어 진입) 후, 리더를 기동하기 전에 편대가 실제로 모일 때까지 기다린다.

round 8 실측: 워밍업(편대 제어 없이 table1 여기만 가하는 구간)이 로봇을 0.2~1.8 m 산개시키고
트랙 유효율을 떨어뜨린 채 제어 단계로 넘긴다. 그 초기조건이 S6 결과를 예측했다
(워밍업말 |e| 0.95/0.15/0.97 -> max_edge 3.33, 1.51/0.23/1.79 -> 40.77). 즉 run 간 10배
편차의 유력한 단일 원인이 '제어를 켜자마자 리더를 출발시킨' 절차였다.

제어·효용 코드는 건드리지 않는다 — 이 스크립트는 순수 대기 게이트다. 기준을 만족하면 exit 0,
상한 시간 안에 못 들면 exit 1(그 run은 '초기조건 불량'으로 기록하고 재시도).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .relative_state import OFFSETS

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']


class SettleWait(Node):
    def __init__(self):
        super().__init__('settle_wait')
        for n, d in [('err_max', 0.5), ('valid_min', 0.9), ('hold', 5.0), ('timeout', 120.0)]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.err_max, self.valid_min = p('err_max'), p('valid_min')
        self.hold, self.timeout = p('hold'), p('timeout')
        self.pos = {}
        self.valid = {r: [] for r in ROBOTS}
        for n in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.pos.__setitem__(
                                         k, np.array([m.pose.pose.position.x,
                                                      m.pose.pose.position.y])), 10)
        for r in ROBOTS:
            self.create_subscription(Float64MultiArray, f'/{r}/state_est',
                                     lambda m, k=r: self.valid[k].append(float(m.data[5])), 10)

    def check(self):
        """(만족?, 최대 편대오차, 로봇별 최근 유효율). 유효율은 최근 100 샘플(=10 s @10 Hz)."""
        if len(self.pos) < 4:
            return False, None, None
        pL = self.pos['platform']
        err = [float(np.linalg.norm((self.pos[r] - pL) - o)) for r, o in zip(ROBOTS, OFFSETS)]
        vr = [float(np.mean(self.valid[r][-100:])) if len(self.valid[r]) >= 20 else 0.0
              for r in ROBOTS]
        ok = max(err) < self.err_max and min(vr) >= self.valid_min
        return ok, max(err), vr


def main():
    rclpy.init()
    g = SettleWait()
    t0 = time.time()
    since = None                                  # 기준을 연속 만족하기 시작한 시각
    last = 0.0
    while time.time() - t0 < g.timeout:
        rclpy.spin_once(g, timeout_sec=0.1)
        ok, err, vr = g.check()
        if err is None:
            continue
        if ok:
            since = since if since is not None else time.time()
            if time.time() - since >= g.hold:
                print(f'settled after {time.time() - t0:.1f}s: max_err={err:.3f} valid={np.round(vr, 3)}')
                sys.exit(0)
        else:
            since = None
        if time.time() - last > 10.0:             # 진행 상황 (진단용)
            last = time.time()
            print(f'  t={time.time() - t0:5.1f}s max_err={err:.3f} valid={np.round(vr, 3)}')
    ok, err, vr = g.check()
    print(f'NOT settled in {g.timeout:.0f}s: max_err={err} valid={vr} -> 초기조건 불량')
    sys.exit(1)
