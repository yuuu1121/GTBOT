import numpy as np

def _pos_vel(X, n):
    return X[:2 * n].reshape(n, 2), X[2 * n:].reshape(n, 2)

def phi_robot(X, i, sc):
    """식 2.7~2.12 + phi7-design §2. dij 규약: 표면거리 |rij|-2Rr (스펙)."""
    pos, vel = _pos_vel(X, sc.n_robots)
    r_star = pos[i] - np.asarray(sc.targets[i])
    v = vel[i]
    nr, nv = float(np.linalg.norm(r_star)), float(np.linalg.norm(v))
    vals = {}
    vals[1] = 0.0 if (nv < sc.eps_guard or nr < sc.eps_guard) else float(r_star @ v) / (nr * nv)
    v_des = sc.v_cruise / (1.0 + np.exp(-10.0 * (nr - 0.2)))
    vals[2] = 1.0 - np.exp(-(((nv - v_des) / (0.8 * v_des)) ** 2))
    vals[3] = np.exp(-((nr / 4.0) ** 2)) * np.exp(-((nr / 6.0) ** 2))
    vals[4] = np.exp(-((nr / 0.05) ** 2)) * np.exp(-((nv / 0.05) ** 2))
    diw = sc.wall_radius - np.linalg.norm(pos[i] - sc.wall_center) - sc.robot_radius
    vals[5] = np.log(1.0 + 6.0 * np.exp(-40.0 * diw))
    dij = min(np.linalg.norm(pos[i] - pos[j]) for j in range(sc.n_robots) if j != i) - 2 * sc.robot_radius
    vals[6] = np.log(1.0 + 10.0 * np.exp(-20.0 * dij))
    if 8 in sc.phi_terms:
        # 리더 반발(φ⁶ 동형). 상대좌표계에서 원점이 리더이므로 |pos[i]|가 리더까지의 중심거리.
        # 리더는 n_robots에 포함되지 않아 φ⁶가 보지 못한다 — 그래서 팔로워가 플랫폼을 관통했다
        # (S6 실측: rel x 1.29 → -0.37 → -1.69, 이후 LiDAR 근접 사각지대 진입).
        d_iL = float(np.linalg.norm(pos[i])) - sc.leader_standoff
        vals[8] = np.log(1.0 + 10.0 * np.exp(-20.0 * d_iL))
    if 7 in sc.phi_terms:
        vals[7] = sum(np.exp(-((np.linalg.norm(pos[a] - pos[b]) - d) ** 2) / sc.sigma ** 2)
                      for (a, b), d in sc.formation.items() if i in (a, b))
    return np.array([vals[j] for j in sc.phi_terms])

def z2_vector(X, sc):
    return np.concatenate([phi_robot(X, i, sc) for i in range(sc.n_robots)])

def surface_distances(X, sc):
    pos, _ = _pos_vel(X, sc.n_robots)
    dr = min(np.linalg.norm(pos[i] - pos[j]) for i in range(sc.n_robots)
             for j in range(i + 1, sc.n_robots)) - 2 * sc.robot_radius
    dw = min(sc.wall_radius - np.linalg.norm(p - sc.wall_center) - sc.robot_radius for p in pos)
    return float(dr), float(dw)
