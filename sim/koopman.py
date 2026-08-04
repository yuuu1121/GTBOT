import numpy as np

def zeta_dim(n_robots: int, m: int, reduced: bool = False) -> int:
    """a(N,m)=(6N+m+1)+24N²+6Nm+m² — phi7-design §7 정정 공식 (901 검산 완료).
    reduced=True: g1(z1d⊗z1d)·g3(z2d⊗z2d) 제거(Task 8b 폴백) → (6N+m+1)+8N²+6Nm (433 검산 완료)."""
    N = n_robots
    if reduced:
        return (6 * N + m + 1) + 8 * N**2 + 6 * N * m
    return (6 * N + m + 1) + 24 * N**2 + 6 * N * m + m**2

def build_zeta(z1d, z2d, U, reduced=False):
    """bilinear ζ (식 2.59~2.68 패턴): [아핀·1차 블록; g1..g5]. z1d=z(1)-z(1)0, z2d=z(2)-z(2)0.
    reduced=True: g1·g3 생략(Task 8b 폴백 — 600샘플 대비 결정계로 축소, U-결합 블록 g4·g5는 보존)."""
    if reduced:
        return np.concatenate([
            z1d, z2d, U, [1.0],
            np.outer(z1d, z2d).ravel(), np.outer(z1d, U).ravel(), np.outer(z2d, U).ravel()])
    return np.concatenate([
        z1d, z2d, U, [1.0],
        np.outer(z1d, z1d).ravel(), np.outer(z1d, z2d).ravel(), np.outer(z2d, z2d).ravel(),
        np.outer(z1d, U).ravel(), np.outer(z2d, U).ravel()])

def build_zeta_linear(z1, z2, U):
    """linear ζ (식 2.40): [z(1); z(2); U]."""
    return np.concatenate([z1, z2, U])
