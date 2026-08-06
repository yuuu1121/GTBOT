"""게이트 S3 — 편대 형성·유지 판정.

실행 시점: koopman_formation이 S2 완료(제어 전환) 로그를 낸 뒤 30 s 수렴 대기 후 실행 —
정상상태 편대유지 측정(사용자 승인 정의, research/sim-results.md 참조).
"""
import json, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .relative_state import OFFSETS, FORMATION

NAMES = ['platform', 'gtbot', 'gtbot2', 'gtbot3']
LEADER_D = [float(np.linalg.norm(o)) for o in OFFSETS]      # 리더-팔로워 목표거리

class GateS3(Node):
    def __init__(self):
        super().__init__('gate_s3')
        self.pos = {}
        for n in NAMES:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.pos.__setitem__(
                                         k, np.array([m.pose.pose.position.x,
                                                      m.pose.pose.position.y])), 10)

def main():
    rclpy.init()
    g = GateS3()
    t0 = time.time()
    log = []                                   # (t, edge_errs[6], min_surf)
    os.makedirs('results', exist_ok=True)
    # 부호 있는 edge 오차 시계열 — 한계사이클(0 중심 진동) vs 바이어스(오프셋) 판별용
    csv = open('results/s3_edges.csv', 'w', buffering=1)
    csv.write('t,' + ','.join(f'e{i}' for i in range(6)) + ',min_surf\n')
    while time.time() - t0 < 180.0:
        rclpy.spin_once(g, timeout_sec=0.1)
        if len(g.pos) < 4:
            continue
        pL = g.pos['platform']
        pf = [g.pos[n] for n in NAMES[1:]]
        signed = [float(np.linalg.norm(pf[i] - pf[j]) - d) for (i, j), d in FORMATION.items()]
        signed += [float(np.linalg.norm(pf[i] - pL) - LEADER_D[i]) for i in range(3)]
        errs = [abs(e) for e in signed]
        dists = [np.linalg.norm(pf[i] - pf[j]) for i in range(3) for j in range(i + 1, 3)]
        dists += [np.linalg.norm(pf[i] - pL) for i in range(3)]
        t, surf = time.time() - t0, float(min(dists) - 0.5)
        log.append((t, errs, surf))                                     # 표면거리(2R=0.5)
        csv.write(f'{t:.3f},' + ','.join(f'{e:.4f}' for e in signed) + f',{surf:.4f}\n')
    csv.close()
    tail = [e for (t, e, _) in log if t > 150.0]
    max_edge_err = float(np.max(tail))
    min_surf = float(min(s for (_, _, s) in log))
    out = {'max_edge_err_tail30s': max_edge_err, 'min_surface_dist': min_surf,
           'n_samples': len(log),
           'gate_s3_pass': bool(max_edge_err < 0.3 and min_surf > 0.0)}
    with open('results/s3_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
