# gtbot_formation/koopman_node.py
import json, os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of
from .relative_state import assemble, make_scenario
from .simpath import ensure
ensure()
from sim.experiment import make_rls, _zeta, operating_point, frozen_1step_eval
from sim.scenario import table1_input
from sim.control import input_objective, solve_input
from sim.utility import z2_vector

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class KoopmanFormation(Node):
    def __init__(self):
        super().__init__('koopman_formation')
        for n, d in [('warmup_steps', 600), ('rate', 10.0), ('results_dir', 'results')]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.warmup_steps, self.results_dir = int(p('warmup_steps')), p('results_dir')
        self.sc = make_scenario()
        self.z10, self.z20 = operating_point(self.sc)
        self.rls = {m: make_rls(self.sc, m) for m in ('linear', 'bilinear')}
        self.k = 0
        self.phase = 'warmup'
        self.X_log, self.U_log = [], []
        self.prev = None                      # (X, z2, U)
        self.odoms = {}                       # name -> (pos2, vel2, t)
        for name in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{name}/odometry',
                                     lambda m, n=name: self.on_odom(n, m), 10)
        self.pubs = [self.create_publisher(Float64MultiArray, f'/{r}/accel_cmd', 10)
                     for r in ROBOTS]
        self.create_timer(1.0 / p('rate'), self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, name, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist.linear
        c, s = np.cos(yaw), np.sin(yaw)
        self.odoms[name] = (np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]),
                            np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y]), self.now())

    def publish_u(self, U):
        for i, pub in enumerate(self.pubs):
            pub.publish(Float64MultiArray(data=[float(U[2 * i]), float(U[2 * i + 1])]))

    def tick(self):
        t = self.now()
        if any(n not in self.odoms or t - self.odoms[n][2] > 0.5 for n in ['platform'] + ROBOTS):
            self.publish_u(np.zeros(6))       # odometry 미비/두절 → 정지 (스펙 에러 처리)
            return
        L = self.odoms['platform'][:2]
        F = [self.odoms[r][:2] for r in ROBOTS]
        X = assemble(L, F)
        z2 = z2_vector(X, self.sc)
        if self.prev is not None:             # 전이 (ζ(k-1) → z2(k))로 RLS 갱신 지속
            Xp, z2p, Up = self.prev
            for m in ('linear', 'bilinear'):
                self.rls[m].update(_zeta(self.sc, m, Xp, z2p, Up, self.z10, self.z20), z2)
        self.k += 1
        if self.phase == 'warmup':
            U = table1_input(self.k) / 8.0
            self.X_log.append(X)
            self.U_log.append(U)
            if self.k >= self.warmup_steps:
                self.finish_warmup()
        else:
            c, _ = input_objective(self.rls['bilinear'].theta, X - self.z10, z2 - self.z20,
                                   self.sc.w_full, 6, reduced=self.sc.reduced_lifting)
            U, status = solve_input(c, self.sc.u_min, self.sc.u_max)
            if status != 'ok':
                self.get_logger().warn(f'LP {status} @k={self.k}')
        self.prev = (X, z2, U)
        self.publish_u(U)

    def finish_warmup(self):
        # frozen_1step_eval은 X_log[k+1] 참조 — 마지막 여기 전이는 평가에서 제외(자기복제 편향 방지)
        X_log = np.array(self.X_log)
        U_log = np.array(self.U_log[:-1])
        rmse = {m: float(np.sqrt(np.mean(
            frozen_1step_eval(self.sc, m, self.rls[m], X_log, U_log) ** 2)))
            for m in ('linear', 'bilinear')}
        out = {'frozen_1step_rmse': rmse,
               'gate_s2_pass': bool(rmse['bilinear'] < rmse['linear']),
               'warmup_steps': self.warmup_steps}
        os.makedirs(self.results_dir, exist_ok=True)
        with open(os.path.join(self.results_dir, 's2_stonefish.json'), 'w') as f:
            json.dump(out, f, indent=1)
        self.get_logger().info(f'S2: {out}')
        self.phase = 'control'

def main():
    rclpy.init()
    rclpy.spin(KoopmanFormation())
