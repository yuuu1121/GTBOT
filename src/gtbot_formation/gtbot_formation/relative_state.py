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
        # QP 정규화 λ — 릴레이(항상 ±u_max) 제거. u = clip(c/λ, ±0.3)이므로 λ가 곧
        # '그래디언트 → 가속 지령' 환산 게인의 역수다. 실측 |c| 중앙값 0.18~0.29
        # (편대오차와 거의 무관 — 릴레이가 크기 정보를 버렸다는 증거).
        # round 8 미세 스윕 {1.0, 1.3, 1.6, 2.0} 실측(|e| 중앙 / 버스트 빈도):
        #   1.0 -> 0.261 / 0.237   1.3 -> 0.381 / 0.438
        #   1.6 -> 0.745 / 0.817   2.0 -> 3.709 / 1.000
        # 단조다 — 파레토 트레이드오프가 없고 λ=1.0이 두 축 모두에서 최적이다. λ를 더 낮추면
        # 릴레이로 회귀(한계사이클 부활), 높이면 복귀 권한 상실. **노브는 여기서 소진됐다**.
        input_reg=1.0,
        # φ¹ 분모 하한 0.05 m/s — 상대속도가 0을 지날 때 그래디언트가 1e4까지 튀어 전 진폭
        # 명령 반전을 유발하던 것을 1/0.05 = 20배 수준으로 유계화한다(round 8 진단).
        # 정상 상대속도는 0.05~0.4 m/s 대역이라 평시에는 비활성인 값이다.
        phi1_v_floor=0.05,
        # robot_radius는 φ⁶의 반응거리 기준(2R)이다. 물리 반지름은 0.22 m지만 0.5로 둔다 —
        # LiDAR 트랙 분리 한계가 물리 충돌보다 먼저 온다: 마커 점군 반경 ~0.3 m + linkage 0.3
        # → 중심간 0.9 m 아래에서 두 로봇이 한 클러스터로 병합돼 트랙이 영구 실종된다
        # (round 5 실측: r1-r2 0.63 m 통과 직후 로봇1 트랙 영구 손실). 2R=1.0이 그 한계를 덮는다.
        # 마스트 재배치(간격 0.15->0.16, 점군 반경 ~0.165->~0.095) 후 병합 한계는 ~0.5 m로
        # 내려가 물리 충돌(0.44 m)이 다시 구속조건이 됐다. 그래도 0.5를 유지한다 — 목표 간격
        # 1.5 m에서 φ⁶ 기여가 4.5e-4로 무시할 수준이라 φ⁷ 평형을 흔들지 않으면서 여유만 준다.
        # ponytail: 지각 분리한계를 물리 반지름 노브로 대신 표현 — 별도 sep_radius 항이 필요해지면 분리.
        robot_radius=0.5, wall_radius=50.0,
        # round 8: φ¹·φ² 가중 완화(2차 노브)를 시도했다가 **되돌렸다**. 진단은 맞았으나
        # (속도항이 위치항 대비 정착 165배·버스트 56배, φ¹은 정규화 코사인이라 스케일 프리)
        # 처방이 틀렸다 — 실측에서 w -1.0 -> -0.1은 권한을 맞춰도(|u| 중앙 0.214 vs 0.222)
        # |e| 중앙 0.261 -> 0.378, 버스트 빈도 0.237 -> 0.385로 악화했다. φ¹·φ²는 잡음이
        # 아니라 **감쇠 그 자체**여서 누르면 루프가 덜 감쇠된다. 원 가중 유지.
        w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0, -2.0]))   # 끝: φ⁸(φ⁶와 동일 가중)
