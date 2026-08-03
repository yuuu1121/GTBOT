import numpy as np
from sim.dynamics import ab_matrices

def test_double_integrator_step():
    A, B = ab_matrices(3, 0.05)
    assert A.shape == (12, 12) and B.shape == (12, 6)
    X = np.zeros(12); X[6] = 1.0            # robot1 vx=1
    U = np.zeros(6); U[1] = 2.0             # robot1 ay=2
    Xn = A @ X + B @ U
    assert np.isclose(Xn[0], 0.05)          # x1 += dt*vx
    assert np.isclose(Xn[1], 0.5 * 0.05**2 * 2.0)   # y1 += 0.5dt²ay
    assert np.isclose(Xn[7], 0.05 * 2.0)    # vy1 += dt*ay
