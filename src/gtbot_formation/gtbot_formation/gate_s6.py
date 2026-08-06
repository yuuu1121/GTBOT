"""게이트 S6 — LiDAR 추정치 기반 편대 형성·유지 판정(gate_s3 복제, gate_s3 원본 불변).

실행 순서(LiDAR 지각 범위 r<5 m 제약의 귀결 — 실측 확인, task-5-report 참조):
워밍업 중 leader가 먼저 출발하면 gtbot이 지각 범위 밖으로 밀려나 state_est가 영구
invalid 되어 k가 안 늘고 S2 전환(제어 진입)이 영원히 오지 않는다. 따라서
`formation.launch.py start_leader:=false`로 기동 → koopman_formation의 S2 완료(제어 전환)
로그 확인 → 이 시점에 `leader_pilot`을 별도 실행(waypoints=[60,0]) → 30 s 정착 대기 →
gate_s6 실행. (GT-odometry 기반 편대(S3)는 이 제약이 없다 — 추정 기반과의 실질 차이.)
표면거리는 platform-gtbot과 gtbot 간을 분리 기록(반지름 합이 다름: 0.47 vs 0.44).
"""
import json, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .relative_state import OFFSETS, FORMATION

NAMES = ['platform', 'gtbot', 'gtbot2', 'gtbot3']
LEADER_D = [float(np.linalg.norm(o)) for o in OFFSETS]      # 리더-팔로워 목표거리(새 OFFSETS=0.866에서 자동)

class GateS6(Node):
    def __init__(self):
        super().__init__('gate_s6')
        self.pos = {}
        for n in NAMES:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.pos.__setitem__(
                                         k, np.array([m.pose.pose.position.x,
                                                      m.pose.pose.position.y])), 10)

def main():
    rclpy.init()
    g = GateS6()
    t0 = time.time()
    log = []                                   # (t, edge_errs[6], min_surf_gg, min_surf_pg)
    os.makedirs('results', exist_ok=True)
    csv = open('results/s6_edges.csv', 'w', buffering=1)
    csv.write('t,' + ','.join(f'e{i}' for i in range(6)) + ',min_surf_gg,min_surf_pg\n')
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
        log.append((t, errs, float(min(surf_gg)), float(min(surf_pg))))
        csv.write(f'{t:.3f},' + ','.join(f'{e:.4f}' for e in signed) +
                  f',{min(surf_gg):.4f},{min(surf_pg):.4f}\n')
    csv.close()
    tail = [e for (t, e, _, _) in log if t > 150.0]
    max_edge_err = float(np.max(tail))
    min_surf_gg = float(min(s for (_, _, s, _) in log))
    min_surf_pg = float(min(s for (_, _, _, s) in log))
    out = {'max_edge_err_tail30s': max_edge_err,
           'min_surface_dist_gg': min_surf_gg, 'min_surface_dist_pg': min_surf_pg,
           'n_samples': len(log),
           'gate_s6_pass': bool(max_edge_err < 0.3 and min_surf_gg > 0.0 and min_surf_pg > 0.0)}
    with open('results/s6_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
