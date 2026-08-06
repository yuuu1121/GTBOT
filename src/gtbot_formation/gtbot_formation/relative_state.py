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
        n_robots=3, phi_terms=(1, 2, 3, 4, 5, 6, 7),
        targets=np.array(OFFSETS), formation=dict(FORMATION), sigma=1.0,
        # dt는 koopman_node의 rate와 짝(dt=1/rate=0.05 @20Hz). analytic_c의 B는 위치감도 dt²/2·
        # 속도감도 dt이므로 dt를 줄이면 그래디언트에서 속도(감쇠) 항 비중이 2/dt로 커진다.
        dt=0.05, u_min=-0.3, u_max=0.3, v_cruise=0.3,
        robot_radius=0.25, wall_radius=50.0,
        w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0]))
