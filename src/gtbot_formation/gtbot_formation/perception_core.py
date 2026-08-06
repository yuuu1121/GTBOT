"""LiDAR 지각 순수함수 — ROS 무관, 합성 클라우드로 테스트."""
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

def marker_split(xyz, h, main_band=(0.0, 0.09), shared_band=(0.10, 0.22), min_sep=0.08):
    """기둥 2개 분리 — 상대 밴딩(캘리브레이션 면역, fix round 1): main_band/shared_band는
    절대 높이가 아니라 이 클러스터의 최고점 h_max 기준 **하강 오프셋**(m)이다.
    중앙 기둥(MastMain) 상단 = h in [h_max-main_band[1], h_max-main_band[0]] (기본: 최상단 9cm).
    오프셋 기둥(MastAux) = h in [h_max-shared_band[1], h_max-shared_band[0]](기본: 10~22cm 아래)
    이면서 중앙에서 min_sep 이상 떨어진 점.
    밴드 폭은 round 4 실측 기하에 맞춘 값이다: MastMain top 0.442 m·MastAux top 0.342 m
    (높이차 0.10)이므로 최상단 9cm는 MastMain 단독, 10~22cm 아래는 두 기둥 공유 구간.
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
