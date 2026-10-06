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
    # φ¹ 분모 |v| 하한(수치 가드, round 9). φ¹은 정규화 코사인이라 grad ∝ 1/|v| —
    # |v|→0에서 유한차분이 1/h 천장까지 폭주한다(실측 p99 = 1e4, 평시 0.3). 하한을 두면
    # 그 발산이 1/floor로 유계가 된다. sc.phi1_v_floor=0(기본)이면 max(nv,0)=nv라
    # 기존 수치 캠페인(E1~E4)과 **비트 동일** — 순수 가산이다.
    nv_g = max(nv, sc.phi1_v_floor)
    vals[1] = 0.0 if (nv < sc.eps_guard or nr < sc.eps_guard) else float(r_star @ v) / (nr * nv_g)
    # 전이 게이트: '스테이션에서 멀다'의 정도. v_des와 φ¹ 게이트가 이 하나를 공유한다
    # (규약·근거는 Scenario 주석 — 기본값이면 게이트=미적용, 원 하드코딩과 비트 동일).
    transit = 1.0 / (1.0 + np.exp(-10.0 * (nr - sc.v_des_r0)))
    if sc.phi1_transit_gate:
        vals[1] *= transit
    v_des = sc.v_cruise * transit
    v_w = 0.8 * max(v_des, sc.v_des_width_floor)
    vals[2] = 1.0 - np.exp(-(((nv - v_des) / v_w) ** 2))
    vals[3] = np.exp(-((nr / 4.0) ** 2)) * np.exp(-((nr / 6.0) ** 2))
    vals[4] = np.exp(-((nr / 0.05) ** 2)) * np.exp(-((nv / 0.05) ** 2))
    diw = sc.wall_radius - np.linalg.norm(pos[i] - sc.wall_center) - sc.robot_radius
    vals[5] = np.log(1.0 + 6.0 * np.exp(-40.0 * diw))
    dij = min(np.linalg.norm(pos[i] - pos[j]) for j in range(sc.n_robots) if j != i) - 2 * sc.robot_radius
    vals[6] = np.log(1.0 + 10.0 * np.exp(-20.0 * dij))
    if 9 in sc.phi_terms:
        # 위치 비례 복원항(round 10). 그래디언트가 ∂(-nr²)/∂pos = -2·r* 로 **오차에 선형**이라
        # QP(u = c/λ)에 정확히 결핍됐던 '크기 있는 복원력'을 공급한다. round 8 분해에서 위치
        # 정보는 그래디언트의 0.6~1.8%뿐이었고, φ¹은 정규화 코사인이라 스케일 프리라서 로봇이
        # 3.5 m 밀려나도 복귀 지령이 ~0이었다(버스트의 정체). round 9는 리더가 정지해도 편대가
        # |e| 0.38~1.45 m로 진동함을 보여 원인이 루프 내부임을 확인했다.
        # r*(=pos[i] - sc.targets[i])는 φ³이 쓰는 것과 동일 규약 — 별도 오프셋 필드를 두지 않는다.
        vals[9] = -float(nr ** 2)
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

def z2_vector(X, sc, h=1e-5):
    z2 = np.concatenate([phi_robot(X, i, sc) for i in range(sc.n_robots)])
    if not sc.grad_lift:
        return z2
    w = np.tile(sc.w_robot, sc.n_robots)
    J = lambda Y: w @ np.concatenate([phi_robot(Y, i, sc) for i in range(sc.n_robots)])
    E = h * np.eye(len(X))
    return np.concatenate([z2, [(J(X + e) - J(X - e)) / (2 * h) for e in E]])

def surface_distances(X, sc):
    pos, _ = _pos_vel(X, sc.n_robots)
    dr = min(np.linalg.norm(pos[i] - pos[j]) for i in range(sc.n_robots)
             for j in range(i + 1, sc.n_robots)) - 2 * sc.robot_radius
    dw = min(sc.wall_radius - np.linalg.norm(p - sc.wall_center) - sc.robot_radius for p in pos)
    return float(dr), float(dw)
