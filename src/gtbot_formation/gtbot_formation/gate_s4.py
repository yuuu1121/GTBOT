"""S4: 지각 품질. ① 정지 60 s — 위치 RMSE(GT 대비) <0.1 m ② gtbot을 전채널 setpoint로
~90°씩 4방위 회전시키며 각 방위 정지 후 마커 헤딩 vs GT yaw 오차 <10°. GT = 시뮬 odometry(평가 전용)."""
import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from gtbot_formation.mixer import yaw_of, wrap

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class GateS4(Node):
    def __init__(self):
        super().__init__('gate_s4')
        self.gt = {}
        self.est = {}
        for n in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.on_gt(k, m), 10)
        for n in ROBOTS:
            self.create_subscription(Float64MultiArray, f'/{n}/state_est',
                                     lambda m, k=n: self.on_est(k, m), 10)
        self.thr = self.create_publisher(Float64MultiArray, '/gtbot/thrusters', 10)

    def on_gt(self, k, m):
        q = m.pose.pose.orientation
        self.gt[k] = (np.array([m.pose.pose.position.x, m.pose.pose.position.y]),
                      yaw_of(q.x, q.y, q.z, q.w))

    def on_est(self, k, m):
        d = m.data
        self.est[k] = (np.array(d[0:2]), d[4], d[5],
                       self.get_clock().now().nanoseconds * 1e-9)

def spin_collect(g, dur):
    t0 = time.time()
    pos_err = {r: [] for r in ROBOTS}
    yaw_err = {r: [] for r in ROBOTS}
    n_valid = n_total = 0
    while time.time() - t0 < dur:
        rclpy.spin_once(g, timeout_sec=0.05)
        if 'platform' not in g.gt:
            continue
        pP, _ = g.gt['platform']
        for r in ROBOTS:
            if r not in g.gt or r not in g.est:
                continue
            (pG, yG), (rel, yE, valid, tE) = g.gt[r], g.est[r]
            n_total += 1
            if not valid or time.time() - tE > 1.0:
                continue
            n_valid += 1
            pos_err[r].append(np.linalg.norm((pG - pP) - rel))
            yaw_err[r].append(abs(wrap(yE - yG)))
    return pos_err, yaw_err, n_valid, max(n_total, 1)

def main():
    rclpy.init()
    g = GateS4()
    pe, _, nv, nt = spin_collect(g, 60.0)                        # ① 정지 위치 정확도
    rmse = {r: float(np.sqrt(np.mean(np.square(v)))) if v else None for r, v in pe.items()}
    yaw_errs = []
    for step in range(4):                                        # ② gtbot 4방위 헤딩
        g.thr.publish(Float64MultiArray(data=[0.25] * 4))
        t0 = time.time()
        while time.time() - t0 < 4.0:                            # ~90° 회전 (실측 조정 노브)
            rclpy.spin_once(g, timeout_sec=0.05)
        g.thr.publish(Float64MultiArray(data=[0.0] * 4))
        time.sleep(3)
        _, ye, _, _ = spin_collect(g, 15.0)
        if ye['gtbot']:
            yaw_errs.append(float(np.degrees(np.median(ye['gtbot']))))
    out = {'pos_rmse': rmse, 'valid_ratio': nv / nt,
           'heading_err_deg_4dir': yaw_errs,
           'gate_s4_pass': bool(all(v is not None and v < 0.1 for v in rmse.values())
                                and len(yaw_errs) == 4
                                and all(e < 10.0 for e in yaw_errs))}
    with open('results/s4_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
