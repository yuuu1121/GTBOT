import numpy as np
from .simpath import ensure
ensure()
from sim.scenario import Scenario

OFFSETS = [(-2.0, 1.0), (-2.0, -1.0), (-3.732, 0.0)]      # 리더 상대, 정삼각형 변 2.0
FORMATION = {(0, 1): 2.0, (0, 2): 2.0, (1, 2): 2.0}

def assemble(leader, followers):
    pL, vL = leader
    pos = np.concatenate([p - pL for p, _ in followers])
    vel = np.concatenate([v - vL for _, v in followers])
    return np.concatenate([pos, vel])

def make_scenario():
    return Scenario(
        n_robots=3, phi_terms=(1, 2, 3, 4, 5, 6, 7),
        targets=np.array(OFFSETS), formation=dict(FORMATION), sigma=1.0,
        dt=0.1, u_min=-0.5, u_max=0.5, v_cruise=0.3,
        robot_radius=0.25, wall_radius=50.0,
        w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0]))
