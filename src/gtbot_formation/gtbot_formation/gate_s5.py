"""S5: 헤딩 정렬. platform_perception + velocity_loop(bearing)×3 가동, 병진 명령 없음.
90 s 관측 중 마지막 30 s — 각 gtbot의 GT 헤딩 vs GT platform 방향(bearing) 오차 median <10°."""
import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gtbot_formation.mixer import yaw_of, wrap

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class GateS5(Node):
    def __init__(self):
        super().__init__('gate_s5')
        self.gt = {}
        for n in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.on_gt(k, m), 10)

    def on_gt(self, k, m):
        q = m.pose.pose.orientation
        self.gt[k] = (np.array([m.pose.pose.position.x, m.pose.pose.position.y]),
                      yaw_of(q.x, q.y, q.z, q.w))

def main():
    rclpy.init()
    g = GateS5()
    t0 = time.time()
    errs = {r: [] for r in ROBOTS}
    while time.time() - t0 < 90.0:
        rclpy.spin_once(g, timeout_sec=0.05)
        if time.time() - t0 < 60.0 or 'platform' not in g.gt:
            continue
        pP, _ = g.gt['platform']
        for r in ROBOTS:
            if r in g.gt:
                pG, yG = g.gt[r]
                bearing = np.arctan2(pP[1] - pG[1], pP[0] - pG[0])
                errs[r].append(abs(np.degrees(wrap(yG - bearing))))
    out = {r: {'median_deg': float(np.median(v)), 'p95_deg': float(np.percentile(v, 95))}
           for r, v in errs.items() if v}
    out['gate_s5_pass'] = bool(len([k for k in out if k in ROBOTS]) == 3
                               and all(out[r]['median_deg'] < 10.0 for r in ROBOTS if r in out))
    with open('results/s5_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
