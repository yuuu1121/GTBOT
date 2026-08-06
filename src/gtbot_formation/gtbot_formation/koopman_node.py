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
from sim.dynamics import ab_matrices
from sim.experiment import make_rls, _zeta, operating_point, frozen_1step_eval, analytic_c
from sim.scenario import table1_input
from sim.control import input_objective, solve_input
from sim.utility import z2_vector

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class KoopmanFormation(Node):
    def __init__(self):
        super().__init__('koopman_formation')
        # rate는 make_scenario의 dt와 짝(1/rate == sc.dt) — analytic_c의 A,B가 dt 기반.
        for n, d in [('warmup_steps', 600), ('rate', 20.0), ('results_dir', 'results'),
                     ('controller', 'analytic'), ('log_csv', ''),
                     ('actuation_delay', 0.16), ('state_source', 'odometry'),
                     ('vel_smooth_alpha', 0.3)]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        path = p('log_csv')                   # velocity_loop과 같은 계측 패턴 (제어 진단용)
        self.log = open(path, 'w', buffering=1) if path else None
        if self.log:
            self.log.write('t,' + ','.join(f'x{i}' for i in range(12)) +
                           ',' + ','.join(f'u{i}' for i in range(6)) + '\n')
        self.warmup_steps, self.results_dir = int(p('warmup_steps')), p('results_dir')
        self.controller = p('controller')         # 'analytic'(기본, 검증된 팔) | 'model'(식별 Θ 실험용)
        self.sc = make_scenario()
        assert abs(1.0 / p('rate') - self.sc.dt) < 1e-9, 'rate와 Scenario.dt 불일치'  # analytic_c·지연 보상 dt 정합 봉인
        self.z10, self.z20 = operating_point(self.sc)
        self.rls = {m: make_rls(self.sc, m) for m in ('linear', 'bilinear')}
        # 지연 보상: 실측 작동기 지연 τ(U→실가속 교차상관 0.16 s)만큼 X를 미리 전파해
        # analytic_c에 넘긴다. 보상 없으면 τ=0.15 s에서 릴레이 한계 사이클이 터진다(round4 대조실험).
        self.tau = float(p('actuation_delay'))
        self.Ad, self.Bd = ab_matrices(len(ROBOTS), self.tau)
        self.u_prev = np.zeros(2 * len(ROBOTS))   # 지연 구간에 이미 발행돼 반영 중인 입력
        self.k = 0
        self.phase = 'warmup'
        self.X_log, self.U_log = [], []
        self.prev = None                      # (X, z2, U)
        self.odoms = {}                       # name -> (pos2, vel2, t)
        for name in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{name}/odometry',
                                     lambda m, n=name: self.on_odom(n, m), 10)
        self.state_source = p('state_source')
        self.vel_alpha = float(p('vel_smooth_alpha'))
        self.vel_ema = {}                     # k -> 지수평활된 rel_vel2 (10 Hz 유한차분 노이즈 완화)
        self.ests = {}                        # k -> (rel_pos2, rel_vel2, valid, t) — lidar 상태원
        if self.state_source == 'lidar':
            for i, r in enumerate(ROBOTS):
                self.create_subscription(Float64MultiArray, f'/{r}/state_est',
                                         lambda m, k=i: self.on_est(k, m), 10)
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

    def on_est(self, k, msg):
        d = msg.data
        valid = bool(d[5])
        v = np.array(d[2:4])
        if valid:                             # invalid 더미(0,0)로 평활 상태를 오염시키지 않음
            v = self.vel_alpha * v + (1 - self.vel_alpha) * self.vel_ema[k] if k in self.vel_ema else v
            self.vel_ema[k] = v
        self.ests[k] = (np.array(d[0:2]), v, valid, self.now())

    def publish_u(self, U):
        self.u_prev = np.asarray(U, dtype=float)   # 지연 보상 전파에 쓰는 '이미 발행된' 입력
        for i, pub in enumerate(self.pubs):
            pub.publish(Float64MultiArray(data=[float(U[2 * i]), float(U[2 * i + 1])]))

    def tick(self):
        # 두절/무효 시 발행을 생략한다(zeros 발행 금지) — zeros 발행은 velocity_loop에서
        # a_cmd=0 갱신으로 해석돼 "직전 v_ref 유지"가 되어 로봇이 마지막 속도로 계속
        # 항해한다(범위 이탈 후 재획득 불가의 원인, S6 LiDAR 실측 확인). 침묵하면
        # velocity_loop의 기존 0.5 s a_cmd 두절 경로가 v_ref를 현재 속도로 리셋하고
        # 추력을 죽여 항력으로 자연 감속한다(검증된 안전 정지, velocity_loop 무수정).
        t = self.now()
        if any(n not in self.odoms or t - self.odoms[n][2] > 0.5 for n in ['platform'] + ROBOTS):
            self.prev = None                  # 두절 갱을 전이로 오인해 RLS에 주입하지 않도록 무효화
            return
        if self.state_source == 'lidar':
            # odometry 두절 가드는 위에서 이미 통과 — 상태 조립에는 미사용, lidar 자체 두절만 가드.
            if any(k not in self.ests or not self.ests[k][2]
                   or t - self.ests[k][3] > 0.5 for k in range(3)):
                self.prev = None
                return
            X = np.concatenate([np.concatenate([self.ests[k][0] for k in range(3)]),
                                np.concatenate([self.ests[k][1] for k in range(3)])])
        else:
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
        elif self.controller == 'analytic':
            # 참 그래디언트 해석적 팔(식별 Θ 불요) — E1~E4 시뮬레이션 캠페인에서 검증된 경로.
            # 식별 Θ 기반(model) LP는 워밍업-목표 간 외삽으로 발산(S3 model 기록 참조), 기각.
            c = analytic_c(self.Ad @ X + self.Bd @ self.u_prev, self.sc, self.sc.w_full)
            U, status = solve_input(c, self.sc.u_min, self.sc.u_max)
            if status != 'ok':
                self.get_logger().warn(f'LP {status} @k={self.k}')
        else:  # 'model' — 식별 Θ 기반 LP (실험/비교용, S3 기각 경로)
            c, _ = input_objective(self.rls['bilinear'].theta, X - self.z10, z2 - self.z20,
                                   self.sc.w_full, 6, reduced=self.sc.reduced_lifting)
            U, status = solve_input(c, self.sc.u_min, self.sc.u_max)
            if status != 'ok':
                self.get_logger().warn(f'LP {status} @k={self.k}')
        self.prev = (X, z2, U)
        self.publish_u(U)
        if self.log:
            self.log.write(','.join(f'{v:.4f}' for v in [t, *X, *U]) + '\n')

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
