import numpy as np

def ab_matrices(n_robots: int, dt: float):
    """이중적분기 A∈R^{4N×4N}, B∈R^{4N×2N} (식 2.3~2.5). X=[pos(2N); vel(2N)]."""
    m = 2 * n_robots
    A = np.eye(2 * m)
    A[:m, m:] = dt * np.eye(m)
    B = np.vstack([0.5 * dt**2 * np.eye(m), dt * np.eye(m)])
    return A, B
