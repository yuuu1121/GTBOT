"""게이트 S6 — LiDAR 추정치 기반 편대 형성·유지 판정(gate_s3 복제, gate_s3 원본 불변).

실행 순서(LiDAR 지각 범위 r<5 m 제약의 귀결 — 실측 확인, task-5-report 참조):
워밍업 중 leader가 먼저 출발하면 gtbot이 지각 범위 밖으로 밀려나 state_est가 영구
invalid 되어 k가 안 늘고 S2 전환(제어 진입)이 영원히 오지 않는다. 따라서
`formation.launch.py start_leader:=false`로 기동 → koopman_formation의 S2 완료(제어 전환)
로그 확인 → **`ros2 run gtbot_formation settle_wait`로 편대 정착을 기다린 뒤**(round 9 추가:
워밍업이 로봇을 산개시킨 채 제어로 넘겨 run 간 10배 편차를 만들던 것을 차단한다. exit 1이면
초기조건 불량이므로 그 run은 폐기·재시도) → `leader_pilot`을 별도 실행(waypoints=[60,0]) →
30 s 대기 → gate_s6 실행. (GT-odometry 기반 편대(S3)는 이 제약이 없다 — 추정 기반과의 실질 차이.)
표면거리는 platform-gtbot과 gtbot 간을 분리 기록(반지름 합이 다름: 0.47 vs 0.44).

판정 기준(round 11 재정의, 사용자 승인): tail30s edge 오차의 **중앙값 < 0.3 이고 p90 < 0.6**,
그리고 무충돌(min_surf_pg > 0, min_surf_gg > 0). 반복성 요건(2회 연속 통과)은 절차로 유지한다.
구 기준인 **max < 0.3**은 참 상태(odometry) 기반 S3가 낸 0.233의 바로 위라 추정 오차 예산이
사실상 0이었다 — 추정 기반 출력피드백에는 분포 기준이 합당하다는 것이 재정의 근거다.
무충돌 항목은 안전 요건이므로 완화 없이 그대로 둔다.

지각 생존 판정 추가(2026-08-14): 되먹임 시험에서 로봇이 스테이션에 머문 채 헤딩만
발산하는 '제자리 스핀'이 나왔다(δ 100~124°, est 유효율 8~20%, 80 s 지속). 위치 기하만
재는 edge 지표는 이를 **통과시킨다** — 게이트의 사각지대였다. 그래서 tail30s에 대해
(1) est 유효율 >= 0.90, (2) 조준 이탈 중앙값 < 20°를 함께 요구한다. 두 항목은
gate_perc_pass·gate_aim_pass로 따로 기록하고 gate_s6_pass에 AND로 들어간다.
**이 변경 이전 S6 결과와는 판정 기준이 다르다**(수치 자체는 비교 가능 — edge 계산 불변).
"""
import json, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of, wrap
from .relative_state import OFFSETS, FORMATION

NAMES = ['platform', 'gtbot', 'gtbot2', 'gtbot3']
LEADER_D = [float(np.linalg.norm(o)) for o in OFFSETS]      # 리더-팔로워 목표거리(새 OFFSETS=0.866에서 자동)

class GateS6(Node):
    def __init__(self):
        super().__init__('gate_s6')
        self.pos = {}
        self.yaw = {}
        self.valid = {}
        for n in NAMES:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.on_odom(k, m), 10)
        # 지각 생존 판정용(2026-08-14 추가) — 아래 gate_perc/gate_aim 참조
        for n in NAMES[1:]:
            self.create_subscription(Float64MultiArray, f'/{n}/state_est',
                                     lambda m, k=n: self.valid.__setitem__(k, m.data[5]), 10)

    def on_odom(self, k, m):
        self.pos[k] = np.array([m.pose.pose.position.x, m.pose.pose.position.y])
        q = m.pose.pose.orientation
        self.yaw[k] = yaw_of(q.x, q.y, q.z, q.w)

def main():
    rclpy.init()
    g = GateS6()
    t0 = time.time()
    log = []                                   # (t, edge_errs[6], min_surf_gg, min_surf_pg)
    perc = []                                  # (t, valid[3], aim_err_deg[3]) — 지각 생존
    os.makedirs('results', exist_ok=True)
    csv = open('results/s6_edges.csv', 'w', buffering=1)
    csv.write('t,' + ','.join(f'e{i}' for i in range(6)) +
              ',min_surf_gg,min_surf_pg,Lx,Ly\n')   # Lx,Ly: 리더 절대위치(리더 실속도 진단용)
    while time.time() - t0 < 180.0:
        rclpy.spin_once(g, timeout_sec=0.1)
        if len(g.pos) < 4:
            continue
        pL = g.pos['platform']
        pf = [g.pos[n] for n in NAMES[1:]]
        signed = [float(np.linalg.norm(pf[i] - pf[j]) - d) for (i, j), d in FORMATION.items()]
        signed += [float(np.linalg.norm(pf[i] - pL) - LEADER_D[i]) for i in range(3)]
        errs = [abs(e) for e in signed]
        surf_gg = [np.linalg.norm(pf[i] - pf[j]) - 0.44 for i in range(3) for j in range(i + 1, 3)]
        surf_pg = [np.linalg.norm(pf[i] - pL) - 0.47 for i in range(3)]
        t = time.time() - t0
        # 지각 생존 지표: 로봇별 est 유효와 조준 이탈 δ(=90°−φ, 판이 얼마나 누웠나).
        # 위치만 보는 edge 지표는 '제자리 스핀'을 통과시킨다(2026-08-14 되먹임 시험).
        vs = [float(g.valid.get(n, 0.0)) for n in NAMES[1:]]
        ds = []
        for i, n in enumerate(NAMES[1:]):
            if n not in g.yaw:
                ds.append(float('nan')); continue
            rel = pf[i] - pL
            ds.append(abs(np.degrees(wrap(g.yaw[n] - np.arctan2(-rel[1], -rel[0])))))
        perc.append((t, vs, ds))
        log.append((t, errs, float(min(surf_gg)), float(min(surf_pg))))
        csv.write(f'{t:.3f},' + ','.join(f'{e:.4f}' for e in signed) +
                  f',{min(surf_gg):.4f},{min(surf_pg):.4f},{pL[0]:.4f},{pL[1]:.4f}\n')
    csv.close()
    tail = np.array([e for (t, e, _, _) in log if t > 150.0])
    max_edge_err = float(np.max(tail))
    med_edge_err = float(np.median(tail))
    p90_edge_err = float(np.percentile(tail, 90))
    min_surf_gg = float(min(s for (_, _, s, _) in log))
    min_surf_pg = float(min(s for (_, _, _, s) in log))
    # --- 지각 생존 판정(2026-08-14 추가) ---
    # 근거: 되먹임 시험에서 로봇이 스테이션에 머문 채 헤딩만 발산하는 '제자리 스핀'이
    # 나왔고(δ 100~124°, est 유효율 8~20%), 위치 기하만 보는 edge 지표는 이를 통과시켰다.
    # 임계값 출처 — valid_min 0.90: 파괴점 캠페인의 통과 최소 91.1% / 실패 최대 85.7%,
    # 혼재대 87~90%(운용 권고는 91% 이상). 정상 런은 100%라 여유가 크다.
    # aim_max 20°: 정상 런의 S5 조준오차가 4.0~6.1°, 스핀 잠김이 100~124°로 그 사이가
    # 비어 있다. 3배 여유를 두고 20°로 잡는다.
    valid_min, aim_max = 0.90, 20.0
    ptail = [(v, d) for (t, v, d) in perc if t > 150.0]
    vr = ([float(np.mean([v[i] for v, _ in ptail])) for i in range(3)] if ptail else [0.0] * 3)
    am = ([float(np.nanmedian([d[i] for _, d in ptail])) for i in range(3)] if ptail else [180.0] * 3)
    gate_perc = bool(ptail) and all(x >= valid_min for x in vr)
    gate_aim = bool(ptail) and all(x < aim_max for x in am)

    out = {'median_edge_err_tail30s': med_edge_err, 'p90_edge_err_tail30s': p90_edge_err,
           'max_edge_err_tail30s': max_edge_err,
           'min_surface_dist_gg': min_surf_gg, 'min_surface_dist_pg': min_surf_pg,
           'n_samples': len(log),
           'valid_ratio_tail30s': dict(zip(NAMES[1:], vr)),
           'aim_err_median_tail30s': dict(zip(NAMES[1:], am)),
           'gate_perc_pass': gate_perc, 'gate_aim_pass': gate_aim,
           'gate_s6_pass': bool(med_edge_err < 0.3 and p90_edge_err < 0.6
                                and min_surf_gg > 0.0 and min_surf_pg > 0.0
                                and gate_perc and gate_aim)}
    with open('results/s6_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
