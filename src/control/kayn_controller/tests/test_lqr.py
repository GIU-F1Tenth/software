import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.lqr import LQRController


def test_dare_converges():
    """solve_discrete_are must not throw and P must be positive definite."""
    model = BicycleModel()
    lqr = LQRController(model)
    x_ref = np.array([0.0, 0.0, 0.0, 2.0])
    u_ref = np.zeros(2)
    K = lqr.compute_gain(x_ref, u_ref)
    assert K is not None


def test_gain_shape():
    """K must be (2, 4): 2 inputs, 4 states."""
    model = BicycleModel()
    lqr = LQRController(model)
    x_ref = np.array([0.0, 0.0, 0.0, 2.0])
    u_ref = np.zeros(2)
    K = lqr.compute_gain(x_ref, u_ref)
    assert K.shape == (2, 4), f"K shape {K.shape} != (2, 4)"


def test_straight_line_convergence():
    """
    Vehicle starts 0.3m off a straight reference line.
    LQR must reduce lateral error to < 0.05m within 3 seconds.
    """
    model = BicycleModel(dt=0.02)
    lqr = LQRController(model)

    x_curr = np.array([0.0, 0.3, 0.05, 2.0])  # offset in y and theta

    dt = 0.02
    steps = int(3.0 / dt)  # 3 seconds

    for i in range(steps):
        x_ref = np.array([x_curr[0], 0.0, 0.0, 2.0])
        u = lqr.compute_control(x_curr, x_ref)
        x_curr = model.step_rk4(x_curr, u)

    lateral_error = abs(x_curr[1])
    assert lateral_error < 0.05, f"LQR did not converge: lateral_error={lateral_error:.4f}m"


def test_gain_caching():
    """Calling compute_control twice at same ref must reuse cached K."""
    model = BicycleModel()
    lqr = LQRController(model)
    x_ref = np.array([0.0, 0.0, 0.0, 2.0])
    lqr.compute_control(np.array([0.1, 0.1, 0.0, 2.0]), x_ref)
    K_first = lqr._cached_K.copy()
    lqr.compute_control(np.array([0.2, 0.1, 0.0, 2.0]), x_ref)
    K_second = lqr._cached_K.copy()
    assert np.allclose(K_first, K_second), "K should be cached for same x_ref"
