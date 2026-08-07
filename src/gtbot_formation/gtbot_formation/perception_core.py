"""LiDAR 지각 순수함수 — ROS 무관, 합성 클라우드로 테스트."""
import itertools

import numpy as np


def lidar_to_world(xyz, yaw, z_offset):
    """platform lidar 포인트(FLU: x 앞, y 왼쪽, z 위) → 월드축 정렬 xy + 수면 위 높이.

    **프레임 실측(round 4, dump_20scan.npz)**: RotatingLidar 클라우드는 로봇 body(NED,
    y 오른쪽·z 아래)가 아니라 ROS 관례인 FLU(z 위)로 나온다. 따라서 body(NED) 성분은
    (x, -y, -z)이고, 월드 정렬은 platform yaw로 **R(+yaw)** 를 곱한다(전치 아님).
    검증: 4개 컨벤션(y미러 × yaw부호) 중 이 조합만 마스트 검출점이 GT odometry에
    0.003~0.014 m로 붙는다(나머지는 0.22~5.6 m). 이전 라운드가 y 미러를 빠뜨린 채
    R(-yaw)를 쓴 것이 "상수 월드 오프셋 ~0.4 m"와 헤딩 미러링(63~128°)의 정체다 —
    platform yaw가 -87° 부근이라 반사+회전이 거의 축퇴돼 그럴듯해 보였을 뿐이다.

    z_offset = 라이다의 수면 위 높이(= 마운트 0.36 m − platform 흘수). 실측 0.198 m로
    마스트 상단(설계 0.442/0.342 m)·선체 상단(0.142 m)이 0.003 m 이내로 재현된다.
    """
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.column_stack([xyz[:, 0], -xyz[:, 1]])          # FLU → body(NED) xy
    return body @ np.array([[c, -s], [s, c]]).T, xyz[:, 2] + z_offset

def cluster_2d(xy, linkage=0.3, min_pts=5):
    """그리디 유니온 클러스터링(O(N²) — 마커 대역 필터 후 수백 점 규모 전제)."""
    n = len(xy)
    label = -np.ones(n, dtype=int)
    cur = 0
    for i in range(n):
        if label[i] >= 0:
            continue
        stack = [i]
        label[i] = cur
        while stack:
            j = stack.pop()
            d = np.linalg.norm(xy - xy[j], axis=1)
            for k in np.nonzero((d < linkage) & (label < 0))[0]:
                label[k] = cur
                stack.append(k)
        cur += 1
    return [np.nonzero(label == c)[0] for c in range(cur)
            if (label == c).sum() >= min_pts]

def marker_split(xyz, h, main_band=(0.0, 0.07), shared_band=(0.10, 0.22), min_sep=0.08):
    """기둥 2개 분리 — 상대 밴딩(캘리브레이션 면역, fix round 1): main_band/shared_band는
    절대 높이가 아니라 이 클러스터의 최고점 h_max 기준 **하강 오프셋**(m)이다.
    중앙 기둥(MastMain) 상단 = h in [h_max-main_band[1], h_max-main_band[0]] (기본: 최상단 9cm).
    오프셋 기둥(MastAux) = h in [h_max-shared_band[1], h_max-shared_band[0]](기본: 10~22cm 아래)
    이면서 중앙에서 min_sep 이상 떨어진 점.
    밴드 폭은 마스트 기하에 맞춘다: 재배치(round 5) 후 두 기둥 상단 높이차가 0.10 → **0.08**로
    줄었으므로(스팬 -0.535..-0.225 vs -0.455..-0.225) main_band 상한을 0.09 → **0.07**로 내린다.
    0.09를 그대로 두면 MastAux 상단(h_max-0.08)이 main 밴드에 섞여 중앙 추정이 aux 쪽으로 끌린다.
    10~22cm 아래는 여전히 두 기둥 공유 구간(aux는 h_max-0.08부터 존재).
    h_max 기준이므로 z_offset 절대 오차와 (프리필터를 뚫고 들어온) 선체 점은 밴드 밖으로
    빠져 자동 배제된다 — 선체 상단은 h_max보다 ~0.30 m 낮다."""
    if len(h) < 3:
        return None
    h_max = h.max()
    top = xyz[(h >= h_max - main_band[1]) & (h <= h_max - main_band[0])]
    if len(top) < 3:
        return None
    center_xy = top[:, :2].mean(axis=0)
    low = xyz[(h >= h_max - shared_band[1]) & (h <= h_max - shared_band[0])]
    if len(low) < 3:
        return None
    d = np.linalg.norm(low[:, :2] - center_xy, axis=1)
    aux = low[d > min_sep]
    if len(aux) < 3:
        return None
    return center_xy, aux[:, :2].mean(axis=0)

def marker_heading(center_xy, aux_xy):
    v = np.asarray(aux_xy) - np.asarray(center_xy)
    return float(np.arctan2(v[1], v[0]))


def kf_step(x, P, z, dt, q, r):
    """트랙별 상수속도 칼만 1스텝. x=[px,py,vx,vy], z=[px,py]. 두 축 등방·독립.

    누적 센트로이드의 유한차분(+EMA)이 내던 rel_vel을 대체한다 — 유한차분은 측정잡음을
    1/dt로 증폭해(σ≈r·√2/dt ≈ 0.35 m/s @ r=0.025, dt=0.1) LP 릴레이의 부호를 노이즈가
    정하게 만들었다(round 4~5 실측: rel_vel SNR≈1, 부호일치 75~91%).

    q = 프로세스 가속 표준편차 [m/s²] — **튜닝 노브**. 이산 백색잡음 가속 모델
    (Bar-Shalom §6.2.2)의 Q를 만든다. 기본 0.5는 상대가속 규모(팔로워 지령 u_max 0.3 +
    플랫폼 기동)에서 잡은 값이고, 편대가 굼뜨면 낮추고 지연이 보이면 올린다.
    r = 측정 표준편차 [m] — S4 실측 위치 RMSE(0.024~0.026 m)에서 0.025.
    """
    F = np.eye(4)
    F[0, 2] = F[1, 3] = dt
    I2 = np.eye(2)
    Q = q ** 2 * np.block([[I2 * dt ** 4 / 4, I2 * dt ** 3 / 2],
                           [I2 * dt ** 3 / 2, I2 * dt ** 2]])
    x = F @ x
    P = F @ P @ F.T + Q
    H = np.zeros((2, 4))
    H[0, 0] = H[1, 1] = 1.0
    S = H @ P @ H.T + r ** 2 * I2
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ (np.asarray(z, dtype=float) - H @ x)
    P = (np.eye(4) - K @ H) @ P
    return x, P


def marker_center(center_xy, aux_xy, offset=(0.0, -0.10)):
    """두 기둥의 중점 -> **로봇 원점**. 마스트가 body offset에 실려 있으면 헤딩으로 되돌린다.

    씬 최종본에서 두 기둥은 body y=-0.10(중앙 상부구조물 상면)에 얹혀 있으므로 중점은
    로봇 원점이 아니라 body (0,-0.10)이다. 보정을 빼면 위치 추정에 0.10 m 상수 편향이
    남아 S4 예산(0.1 m)을 통째로 먹는다. offset은 캘리브레이션 노브 — 마스트를 옮기면
    여기만 다시 잰다.
    """
    c, a = np.asarray(center_xy, dtype=float), np.asarray(aux_xy, dtype=float)
    yaw = marker_heading(c, a)
    ct, st = np.cos(yaw), np.sin(yaw)
    return 0.5 * (c + a) - np.array([[ct, -st], [st, ct]]) @ np.asarray(offset, dtype=float)


def assign_tracks(dets_xy, tracks, gate):
    """검출 <-> 트랙 **전역 최적 배정**. 반환: 트랙별 검출 인덱스(또는 None).

    기존 탐욕(argmin 후 `out[k] is None`이면 채택)은 이미 점유된 트랙에 최근접인 클러스터를
    **그냥 버려서** 다른 트랙을 굶겼다. 그리고 무효 트랙의 앵커는 갱신되지 않아 고착되므로
    한 번 굶은 트랙은 영구 미아가 된다(round 11 실측: platform_perception만 재시작해 앵커를
    초기화하자 **동일 장면에서 valid 집합이 바뀜** -> 탐지가 아니라 연관 계층 결함).

    트랙 3개라 배정 후보가 (n_det+1)^3로 작아 순열 전수로 최적해를 고른다 — solver 불필요.
    목적은 **사전식(lexicographic)**이다: 먼저 배정 개수를 최대화하고, 같은 개수 안에서
    거리 합을 최소화한다. 미배정 벌점을 gate로만 두면 '한 트랙을 버리고 다른 트랙에 더 가까운
    검출을 주는' 쪽이 총합에서 이겨 기아가 되살아난다(실제로 그렇게 실패했다) — 벌점을 게이트
    합보다 크게 잡아 개수 항이 항상 지배하게 한다.
    """
    BIG = 1e3                                            # 미배정 벌점 (거리 합 최대치보다 큼)
    n = len(dets_xy)
    best, best_cost = (None,) * len(tracks), None
    for combo in itertools.product([None] + list(range(n)), repeat=len(tracks)):
        used = [c for c in combo if c is not None]
        if len(set(used)) != len(used):                  # 한 검출을 두 트랙에 줄 수 없다
            continue
        cost, ok = 0.0, True
        for k, j in enumerate(combo):
            if j is None:
                cost += BIG
                continue
            d = float(np.linalg.norm(np.asarray(dets_xy[j]) - np.asarray(tracks[k])))
            if d >= gate:
                ok = False
                break
            cost += d
        if ok and (best_cost is None or cost < best_cost):
            best, best_cost = combo, cost
    return list(best)
