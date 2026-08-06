import numpy as np
from .simpath import ensure
ensure()
from sim.scenario import Scenario

OFFSETS = [(0.866, 0.0), (-0.433, 0.750), (-0.433, -0.750)]      # platform 중심 정삼각형, gtbot 간 1.5 m
FORMATION = {(0, 1): 1.5, (0, 2): 1.5, (1, 2): 1.5}

def assemble(leader, followers):
    pL, vL = leader
    pos = np.concatenate([p - pL for p, _ in followers])
    vel = np.concatenate([v - vL for _, v in followers])
    return np.concatenate([pos, vel])

def make_scenario():
    return Scenario(
        # φ⁸(리더 반발) 추가 — 사용자 결정(round 5): 반경 0.866 m를 유지한 채 플랫폼 관통을
        # 막는다. leader_standoff 0.47 = platform 0.25 + gtbot 0.22 (표면 여유 0.396 m).
        n_robots=3, phi_terms=(1, 2, 3, 4, 5, 6, 7, 8), leader_standoff=0.47,
        targets=np.array(OFFSETS), formation=dict(FORMATION), sigma=1.0,
        # dt는 koopman_node의 rate와 짝(dt=1/rate=0.05 @20Hz). analytic_c의 B는 위치감도 dt²/2·
        # 속도감도 dt이므로 dt를 줄이면 그래디언트에서 속도(감쇠) 항 비중이 2/dt로 커진다.
        dt=0.05, u_min=-0.3, u_max=0.3, v_cruise=0.3,
        # robot_radius는 φ⁶의 반응거리 기준(2R)이다. 물리 반지름은 0.22 m지만 0.5로 둔다 —
        # LiDAR 트랙 분리 한계가 물리 충돌보다 먼저 온다: 마커 점군 반경 ~0.3 m + linkage 0.3
        # → 중심간 0.9 m 아래에서 두 로봇이 한 클러스터로 병합돼 트랙이 영구 실종된다
        # (round 5 실측: r1-r2 0.63 m 통과 직후 로봇1 트랙 영구 손실). 2R=1.0이 그 한계를 덮는다.
        # 마스트 재배치(간격 0.15->0.16, 점군 반경 ~0.165->~0.095) 후 병합 한계는 ~0.5 m로
        # 내려가 물리 충돌(0.44 m)이 다시 구속조건이 됐다. 그래도 0.5를 유지한다 — 목표 간격
        # 1.5 m에서 φ⁶ 기여가 4.5e-4로 무시할 수준이라 φ⁷ 평형을 흔들지 않으면서 여유만 준다.
        # ponytail: 지각 분리한계를 물리 반지름 노브로 대신 표현 — 별도 sep_radius 항이 필요해지면 분리.
        robot_radius=0.5, wall_radius=50.0,
        w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0, -2.0]))   # 끝: φ⁸(φ⁶와 동일 가중)
