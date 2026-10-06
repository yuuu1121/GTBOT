# gtbot_formation/koopman_node.py
import json, os
from dataclasses import replace
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
from sim.experiment import (make_rls, _zeta, operating_point, frozen_1step_eval, analytic_c,
                            nominal_theta, model_scenario, MODEL_ARM_FIT)
from sim.lifted import build_mpc_model, in_trust_region
from sim.scenario import table1_input
from sim.control import input_objective, solve_input
from sim.utility import z2_vector

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class KoopmanFormation(Node):
    def __init__(self):
        super().__init__('koopman_formation')
        # rate는 make_scenario의 dt와 짝(1/rate == sc.dt) — analytic_c의 A,B가 dt 기반.
        for n, d in [('warmup_steps', 600), ('rate', 20.0), ('results_dir', 'results'),
                     ('controller', 'mpc'), ('log_csv', ''),
                     # mpc 팔 노브(sim/lifted 주석): 지평 H, 입력 변화 패널티 ρ(포화율↔코너 이득), 사전 포화 좌표 s,
                     # 식별 시드, Θ 캐시 폴더(''=캐시 끔). 2026-10-06 Stonefish 4×4 사각 값이 기본.
                     ('mpc_horizon', 3), ('mpc_rho', 3.0), ('mpc_sat', 1.5), ('mpc_fit_seed', 0),
                     ('mpc_cache_dir', os.path.expanduser('~/.cache/gtbot')),
                     ('actuation_delay', 0.16), ('state_source', 'odometry'),
                     # 1.0 = 평활 무효(통과). round 6에서 rel_vel이 platform_perception의
                     # 트랙 KF 출력으로 바뀌어 여기서 또 EMA를 걸면 지연만 더한다.
                     ('vel_smooth_alpha', 1.0), ('est_hold', 1.0), ('excite_div', 8.0),
                     # require_odom(2026-08-10): odometry 신선도 가드·GT 진단 로그의
                     # 사용 여부. 시뮬 True(기존 동작). 실기에는 odometry가 없어 이
                     # 가드가 tick을 영구 차단하므로 hardware_platform.launch에서 False.
                     # lidar 모드의 제어 상태 조립은 어차피 est만 쓴다(odometry 미사용).
                     ('require_odom', True),
                     ('est_stale_stop', 5.0)]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        path = p('log_csv')                   # velocity_loop과 같은 계측 패턴 (제어 진단용)
        self.log = open(path, 'w', buffering=1) if path else None
        if self.log:
            # pub: 이 틱에 accel_cmd를 발행했는지(0=게이트로 침묵). vk: 로봇 k의 추정 유효·신선.
            # gk: odometry로 조립한 참값 X(추정 오차를 같은 틱에서 직접 재기 위한 진단 열).
            self.log.write('t,' + ','.join(f'x{i}' for i in range(12)) +
                           ',' + ','.join(f'u{i}' for i in range(6)) +
                           ',pub,v0,v1,v2,' + ','.join(f'g{i}' for i in range(12)) + '\n')
        self.warmup_steps, self.results_dir = int(p('warmup_steps')), p('results_dir')
        self.controller = p('controller')         # 'mpc'(기본: 불변성 보강 사전 Θ, H스텝) | 'analytic'(참 기울기 기준선) | 'model'(원논문 꼴 Θ 1-step)
        self.sc = make_scenario()
        # 식별용 시나리오 = 제어용에서 φ⁹만 뺀 것. φ⁹ = -nr²는 다른 φ(유계 0~2)와 달리
        # **무계**라 리프팅 z2에 들어가면 1-step 예측 RMSE를 악화시켜 S2(bilinear 리프팅 품질
        # 지표)를 직접 때린다(round 10 실측: φ⁹ 활성 후 실패율 30% -> 5회 중 3회). φ⁹은
        # 목적함수에만 필요하므로(복원력 공급) 식별 경로에서 제외한다 — 원인 제거.
        keep = [i for i, t in enumerate(self.sc.phi_terms) if t != 9]
        self.sc_id = replace(self.sc, phi_terms=tuple(self.sc.phi_terms[i] for i in keep),
                             w_robot=self.sc.w_robot[keep])
        assert abs(1.0 / p('rate') - self.sc.dt) < 1e-9, 'rate와 Scenario.dt 불일치'  # analytic_c·지연 보상 dt 정합 봉인
        self.z10, self.z20 = operating_point(self.sc_id)
        self.rls = {m: make_rls(self.sc_id, m) for m in ('linear', 'bilinear')}
        # model 팔의 제어용 bilinear 모델 — S2용 self.rls(Θ0=0, φ⁹ 제외)와 별개다.
        # 원논문 형태(Zhao & Tao): 사전은 효용항 φ만(그래디언트 ψ 없음), ζ는 전체 bilinear,
        # Θ는 공칭 플랜트 데이터의 배치 적합 + 궤적 위 재적합(nominal_theta·MODEL_ARM_FIT 주석),
        # 제어는 1-step QP. 제어 목적함수는 φ⁹(복원력)가 필요하므로 φ⁹는 두고 φ⁴만 뺀다
        # (model_scenario). 그래디언트를 사전에 덧붙인 변종(Scenario.grad_lift)은 c가 정확하나
        # 사전이 J의 미분을 이미 담아 동어반복이라 노드에서는 쓰지 않는다(draft 설명 노트 §10).
        # Θ0=0 + 워밍업 표본으로는 발산했다(S3 기록). Θ는 제어 중 **동결**이다 — 제어 중 RLS
        # 갱신은 공칭 플랜트에서도 발산했다(지연 정렬·배치 사전분포를 줘도 6/6).
        if self.controller == 'model':
            self.sc_c = model_scenario(self.sc)
            self.theta_c = nominal_theta(self.sc_c, **MODEL_ARM_FIT)
            self.z10c, self.z20c = operating_point(self.sc_c)
            self.get_logger().info(f'model 팔: 공칭 bilinear Θ 적합 완료 (dim {self.theta_c.shape[0]})')
        # mpc 팔: 사전 [1; poly3(포화 좌표); φ; φ⊗poly2](sim/lifted 주석)로 다스텝 예측이 서는 Θ를 공칭
        # 플랜트에서 적합하고(약 10 s), H스텝 효용을 수반 역전파 MPC(입력 변화 패널티 ρ)로 최대화한다.
        # Stonefish 4×4 사각: edge 중앙 0.151 vs analytic 0.168 m. 틱당 12~27 ms(20 Hz 예산 50 ms).
        if self.controller == 'mpc':
            self.H, self.rho = int(p('mpc_horizon')), float(p('mpc_rho'))
            self.lifted = build_mpc_model(self.sc, seed=int(p('mpc_fit_seed')), sat=float(p('mpc_sat')),
                                          cache_dir=p('mpc_cache_dir') or None)
            self.plan = np.zeros((self.H, 2 * len(ROBOTS))); self.n_fallback = 0
            self.get_logger().info(f'mpc 팔: 리프팅 모델 적합 완료 (dim {self.lifted.n}, H={self.H})')
        # 지연 보상: 실측 작동기 지연 τ(U→실가속 교차상관 0.16 s)만큼 X를 미리 전파해
        # analytic_c에 넘긴다. 보상 없으면 τ=0.15 s에서 릴레이 한계 사이클이 터진다(round4 대조실험).
        self.tau = float(p('actuation_delay'))
        self.Ad, self.Bd = ab_matrices(len(ROBOTS), self.tau)
        self.u_prev = np.zeros(2 * len(ROBOTS))   # 지연 구간에 이미 발행돼 반영 중인 입력
        self.k = 0
        self.phase = 'warmup'
        # plate 부트스트랩 상태기계(2026-08-09, 사용자 승인 3안): lidar 모드에서 전 로봇의
        # est가 **연속 3 s 유효**할 때까지 koopman은 완전 침묵한다. 간헐 깜빡임이 워밍업
        # 가진을 버스트로 깨워 로봇을 흔들고, 그 흔들림이 다시 검출을 깨는 3-체제 스위칭
        # (워밍업/bearing/폴백 혼합)이 실측 확인됨 — 침묵 유지가 평수면에서 로봇을 정지시켜
        # 검출기가 정적 장면을 보게 한다. odometry 모드는 무관(항상 boot_done).
        self.boot_done = (p('state_source') != 'lidar')
        self.boot_ok_since = None
        self.X_log, self.U_log = [], []
        self.prev = None                      # (X, z2, U)
        self.odoms = {}                       # name -> (pos2, vel2, t)
        for name in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{name}/odometry',
                                     lambda m, n=name: self.on_odom(n, m), 10)
        self.state_source = p('state_source')
        self.vel_alpha = float(p('vel_smooth_alpha'))
        self.vel_ema = {}                     # k -> 지수평활된 rel_vel2 (10 Hz 유한차분 노이즈 완화)
        self.est_hold = float(p('est_hold'))
        self.excite_div = float(p('excite_div'))  # 워밍업 가진 스케일 (8=원본, lidar 모드는 launch에서 완화)
        self.est_stale_stop = float(p('est_stale_stop'))
        self.require_odom = bool(p('require_odom'))
        self.ests = {}                        # k -> (rel_pos2, rel_vel2, valid, t) — lidar 상태원
        self.ok = {}                          # k -> (rel_pos2, rel_vel2, t) — 마지막 '유효' 추정(홀드)
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
        t = self.now()
        self.ests[k] = (np.array(d[0:2]), v, valid, t)
        if valid:
            self.ok[k] = (np.array(d[0:2]), v, t)

    def publish_u(self, U, mask=(True, True, True)):
        U = np.asarray(U, dtype=float).copy()
        for i, ok in enumerate(mask):
            if not ok:
                U[2 * i:2 * i + 2] = 0.0       # 침묵한 로봇은 곧 두절 경로로 정지 — 전파도 0
        self.u_prev = U                        # 지연 보상 전파에 쓰는 '이미 발행된' 입력
        for i, pub in enumerate(self.pubs):
            if mask[i]:
                pub.publish(Float64MultiArray(data=[float(U[2 * i]), float(U[2 * i + 1])]))

    def logrow(self, t, X, U, pub):
        if not self.log:
            return
        v = [float(k in self.ests and self.ests[k][2] and t - self.ests[k][3] <= 0.5)
             for k in range(3)]
        if all(n in self.odoms for n in ['platform'] + ROBOTS):
            gt = assemble(self.odoms['platform'][:2], [self.odoms[r][:2] for r in ROBOTS])
        else:
            gt = np.zeros(12)                  # 실기(require_odom=False): GT 진단열 없음
        self.log.write(','.join(f'{x:.4f}' for x in [t, *X, *U, pub, *v, *gt]) + '\n')

    def control(self, X):
        """제어 단계의 U. 세 팔 공통으로 지연 보상 상태 Xd 를 쓴다."""
        Xd = self.Ad @ X + self.Bd @ self.u_prev
        arm = self.controller
        if arm == 'mpc' and not in_trust_region(Xd, self.sc):
            # 식별 영역(자리 0.45 m·속도 0.3 m/s) 밖은 analytic — 포화 좌표로 밖까지 덮으려 했으나 초기조건 시드
            # 1~5 에서 전부 발산해 되돌렸다(sim/lifted identification_data 주석). 사각 주행 중에는 발동 0.
            arm = 'analytic'; self.n_fallback += 1
        if self.controller == 'mpc' and self.k % 1200 == 0:
            self.get_logger().info(f'mpc 폴백 누적 {self.n_fallback}/{self.k} 틱')
        if arm == 'analytic':
            # 참 그래디언트 해석적 팔(식별 Θ 불요) — E1~E4 시뮬레이션 캠페인에서 검증된 경로.
            # Θ0=0 + 워밍업 표본의 model 팔은 외삽으로 발산했다(S3 model 기록) — 현 model 팔은 공칭 Θ0.
            c = analytic_c(Xd, self.sc, self.sc.w_full)
        elif arm == 'mpc':  # 리프팅 모델로 H스텝 효용 최대화, 첫 입력만 발행(워밍스타트 = 이전 계획 한 칸 밀기)
            self.plan = self.lifted.mpc(self.lifted.D.lift(Xd), self.H, np.vstack([self.plan[1:], self.plan[-1:]]),
                                        self.sc.input_reg, self.sc.u_min, self.sc.u_max, rho=self.rho, u_prev=self.u_prev)
            U = self.plan[0]
            if not np.isfinite(U).all():
                U = np.zeros_like(U); self.plan[:] = 0.0; self.get_logger().warn(f'MPC nan_guard @k={self.k}')
            return U
        else:  # 'model' — bilinear Koopman 모델 Θ가 c를 낸다
            c, _ = input_objective(self.theta_c, Xd - self.z10c,
                                   z2_vector(Xd, self.sc_c) - self.z20c, self.sc_c.w_full, 6)
        U, status = solve_input(c, self.sc.u_min, self.sc.u_max, reg=self.sc.input_reg)
        if status != 'ok':
            self.get_logger().warn(f'LP {status} @k={self.k}')
        return U

    def tick(self):
        # 두절/무효 시 발행을 생략한다(zeros 발행 금지) — zeros 발행은 velocity_loop에서
        # a_cmd=0 갱신으로 해석돼 "직전 v_ref 유지"가 되어 로봇이 마지막 속도로 계속
        # 항해한다(범위 이탈 후 재획득 불가의 원인, S6 LiDAR 실측 확인). 침묵하면
        # velocity_loop의 기존 0.5 s a_cmd 두절 경로가 v_ref를 현재 속도로 리셋하고
        # 추력을 죽여 항력으로 자연 감속한다(검증된 안전 정지, velocity_loop 무수정).
        t = self.now()
        if self.require_odom and any(
                n not in self.odoms or t - self.odoms[n][2] > 0.5
                for n in ['platform'] + ROBOTS):
            self.prev = None                  # 두절 갱을 전이로 오인해 RLS에 주입하지 않도록 무효화
            return
        mask = (True, True, True)
        if self.state_source == 'lidar' and not self.boot_done:
            fresh = all(k in self.ests and self.ests[k][2]
                        and t - self.ests[k][3] < self.est_hold for k in range(3))
            if fresh:
                if self.boot_ok_since is None:
                    self.boot_ok_since = t
                if t - self.boot_ok_since >= 3.0:
                    self.boot_done = True
                    self.get_logger().info('bootstrap: est 연속 유효 3 s — 워밍업 시작')
            else:
                self.boot_ok_since = None
            if not self.boot_done:
                self.prev = None
                self.logrow(t, np.zeros(12), np.zeros(6), 0.0)
                return
        if self.state_source == 'lidar':
            # odometry 두절 가드는 위에서 이미 통과 — 상태 조립에는 미사용, lidar 자체 두절만 가드.
            # 로봇별 홀드: 마지막 '유효' 추정을 est_hold 동안 쓰고, 만료된 로봇만 침묵시킨다.
            # 전원 침묵(구 동작)은 한 대의 일시 실명이 세 대 전부를 velocity_loop 두절로 정지시켜
            # 플랫폼이 떠난 뒤 영구 미복구가 된다(S6 실측: 로봇0이 0.34 m 근접 = LiDAR 사각지대
            # 진입 → 220 s 전원 침묵 → 3대 정지, 편대오차 22 m 발산).
            if any(k not in self.ok for k in range(3)):
                self.prev = None
                self.logrow(t, np.zeros(12), np.zeros(6), 0.0)
                return
            if any(t - self.ok[k][2] > self.est_stale_stop for k in range(3)):
                # 영구 손실 최후 방어선. mask는 **발행만** 막고 X 조립에는 self.ok[k]의 마지막
                # 값이 그대로 쓰이므로, 한 대가 영구 손실되면 그 좌표가 관측 시점에 고정된 채
                # 건강한 두 대의 φ⁶/φ⁷/φ⁸ 그래디언트에 계속 들어간다 — 유령 위치 기준 최적화다.
                # 편대 상대상태가 무결하지 않으면 최적화 지속이 무의미하므로 전원 침묵으로
                # 전환한다. est_stale_stop(5 s) > platform_perception의 트랙별 재획득(4 s)이라
                # 재획득이 먼저 발화할 기회를 준 뒤의 최후 방어선이다.
                self.prev = None
                self.logrow(t, np.zeros(12), np.zeros(6), 0.0)
                return
            mask = tuple(t - self.ok[k][2] <= self.est_hold for k in range(3))
            if not any(mask):
                self.prev = None
                self.logrow(t, np.zeros(12), np.zeros(6), 0.0)
                return
            if not all(mask):
                self.prev = None              # 만료 홀드값이 섞인 전이를 RLS에 주입하지 않음
            X = np.concatenate([np.concatenate([self.ok[k][0] for k in range(3)]),
                                np.concatenate([self.ok[k][1] for k in range(3)])])
        else:
            L = self.odoms['platform'][:2]
            F = [self.odoms[r][:2] for r in ROBOTS]
            X = assemble(L, F)
        if self.phase == 'warmup' and not all(mask):
            # I-1: 마스킹 틱은 워밍업 표본에서 **통째로** 건너뛴다. X에는 만료된 홀드 좌표가
            # 섞이고, U에는 publish_u가 실제로는 발행하지 않은 로봇의 입력이 그대로 남아
            # '가해지지 않은 입력'이 식별 데이터가 된다 — 둘 다 frozen_1step_eval을 거쳐
            # S2 판정을 만든다. self.k도 올리지 않아 여기 수열(table1_input)과 실제 인가가
            # 어긋나지 않는다. odometry 모드는 mask가 항상 전부 True라 이 분기에 오지 않는다.
            self.prev = None
            self.logrow(t, X, np.zeros(6), float(sum(mask)) / 3.0)
            return
        z2 = z2_vector(X, self.sc_id)   # 식별용 리프팅(φ⁹ 제외)
        # RLS 갱신은 식별 Θ를 읽는 경로가 살아 있을 때만 돈다(2026-08-17 계측).
        # Θ를 읽는 곳은 둘뿐이다 — 아래 'model' 분기와 finish_warmup의 S2 게이트.
        # 기본 팔인 'analytic'은 참 그래디언트를 쓰므로 제어 단계에서 Θ를 아무도
        # 읽지 않는데, bilinear의 P가 1477x1477 = 17.5 MB라 매 틱 outer product 생성·
        # 뺄셈·대칭화로 이 배열을 여러 번 훑는다(실측 틱당 11.6 ms, 20 Hz 예산의 23%).
        # 660 s 임무면 13,200틱이 통째로 죽은 계산이었고, 그 부하가 렌더 경로를 굶겨
        # LiDAR 발행률을 5 Hz -> 1 Hz로 끌어내렸다(A1/A2 대조). S2는 워밍업 산출물이라
        # 영향받지 않는다.
        # 2026-10-06 정리: model 팔(theta_c)·mpc 팔(lifted)은 self.rls 를 읽지 않는다. 워밍업 RLS 는 S2 게이트
        # (bilinear < linear 잔차, 식별 가능성 확인)와 run 스크립트의 '제어 진입' 신호("S2: {" 로그)로만 남긴다 —
        # 워밍업 200틱 동안만 돌아 비용이 없고, 절차를 바꾸면 결과 재현 스크립트가 깨진다.
        needs_theta = self.phase == 'warmup'
        if self.prev is not None and needs_theta:   # 전이 (ζ(k-1) → z2(k))로 RLS 갱신
            Xp, z2p, Up = self.prev
            for m in ('linear', 'bilinear'):
                self.rls[m].update(_zeta(self.sc_id, m, Xp, z2p, Up, self.z10, self.z20), z2)
        self.k += 1
        if self.phase == 'warmup':
            U = table1_input(self.k) / self.excite_div
            self.X_log.append(X)
            self.U_log.append(U)
            if self.k >= self.warmup_steps:
                self.finish_warmup()
        else:
            U = self.control(X)
        # I-2: 오염 전이는 다음 틱에도 재주입되지 않도록 여기서도 막는다. 161행의 prev=None은
        # 그 틱의 갱신만 막았고, 이 줄이 오염된 X로 prev를 다시 채워 다음 틱에 흘려보냈다.
        self.prev = (X, z2, U) if all(mask) else None
        self.publish_u(U, mask)
        self.logrow(t, X, U, float(sum(mask)) / 3.0)

    def finish_warmup(self):
        # frozen_1step_eval은 X_log[k+1] 참조 — 마지막 여기 전이는 평가에서 제외(자기복제 편향 방지)
        X_log = np.array(self.X_log)
        U_log = np.array(self.U_log[:-1])
        rmse = {m: float(np.sqrt(np.mean(
            frozen_1step_eval(self.sc_id, m, self.rls[m], X_log, U_log) ** 2)))
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
