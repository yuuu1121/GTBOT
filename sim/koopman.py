import numpy as np

def zeta_dim(n_robots: int, m: int) -> int:
    """a(N,m)=(6N+m+1)+24N²+6Nm+m² — phi7-design §7 정정 공식 (901 검산 완료)."""
    N = n_robots
    return (6 * N + m + 1) + 24 * N**2 + 6 * N * m + m**2

def build_zeta(z1d, z2d, U):
    """bilinear ζ (식 2.59~2.68 패턴): [아핀·1차 블록; g1..g5]. z1d=z(1)-z(1)0, z2d=z(2)-z(2)0."""
    return np.concatenate([
        z1d, z2d, U, [1.0],
        np.outer(z1d, z1d).ravel(), np.outer(z1d, z2d).ravel(), np.outer(z2d, z2d).ravel(),
        np.outer(z1d, U).ravel(), np.outer(z2d, U).ravel()])

def build_zeta_linear(z1, z2, U):
    """linear ζ (식 2.40): [z(1); z(2); U]."""
    return np.concatenate([z1, z2, U])
