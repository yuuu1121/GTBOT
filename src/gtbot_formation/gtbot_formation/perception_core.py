"""LiDAR 지각 순수함수 — ROS 무관, 합성 클라우드로 테스트."""
import numpy as np

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

def marker_split(xyz, h, main_band=(0.0, 0.05), shared_band=(0.06, 0.14), min_sep=0.08):
    """기둥 2개 분리 — 상대 밴딩(캘리브레이션 면역, fix round 1): main_band/shared_band는
    절대 높이가 아니라 이 클러스터의 최고점 h_max 기준 **하강 오프셋**(m)이다.
    중앙 기둥(MastMain) 상단 = h in [h_max-main_band[1], h_max-main_band[0]] (기본: 최상단 5cm).
    오프셋 기둥(MastAux) = h in [h_max-shared_band[1], h_max-shared_band[0]](기본: 6~14cm 아래,
    두 기둥 높이차 0.08m가 판별 기준) 이면서 중앙에서 min_sep 이상 떨어진 점.
    h_max 기준이므로 z_water_offset/z_sign의 절대 오차가 그대로 상쇄된다."""
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
