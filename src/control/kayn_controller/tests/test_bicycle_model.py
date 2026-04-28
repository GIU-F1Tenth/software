import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel


def test_circle_open_loop():
    """Constant steering and speed should trace a circle.
    Radius = v / (v * tan(delta) / L) = L / tan(delta).
    """
    model = BicycleModel(L=0.33, dt=0.01)
    v = 2.0
    delta = 0.2  # rad
    expected_radius = model.L / np.tan(delta)  # ~1.62 m

    x = np.array([0.0, 0.0, 0.0, v])
    u = np.array([delta, 0.0])

    # Integrate for one full circle
    period = 2 * np.pi * expected_radius / v
    steps = int(period / model.dt)
    for _ in range(steps):
        x = model.step_rk4(x, u)

    # Should return close to origin
    assert np.linalg.norm(x[:2]) < 0.15, f"Circle not closed: pos={x[:2]}"


def test_linearize_shapes():
    """A_d must be (4,4), B_d must be (4,2)."""
    model = BicycleModel()
    x_ref = np.array([0.0, 0.0, 0.0, 2.0])
    u_ref = np.array([0.0, 0.0])
    A_d, B_d = model.linearize(x_ref, u_ref)
    assert A_d.shape == (4, 4)
    assert B_d.shape == (4, 2)


def test_normalize_angle():
    model = BicycleModel()
    assert abs(model.normalize_angle(2 * np.pi + 0.1) - 0.1) < 1e-9
    assert abs(model.normalize_angle(-2 * np.pi - 0.1) - (-0.1)) < 1e-9


def test_linearize_values():
    """Jacobian entries should match analytic partial derivatives."""
    model = BicycleModel(L=0.33, dt=0.02)
    v, theta, delta = 3.0, 0.0, 0.0
    x_ref = np.array([0.0, 0.0, theta, v])
    u_ref = np.array([delta, 0.0])
    A_d, B_d = model.linearize(x_ref, u_ref)
    # A_d = I + A*dt, check off-diagonal entries
    assert abs(A_d[0, 3] - np.cos(theta) * model.dt) < 1e-10  # df_px/dv * dt
    assert abs(A_d[1, 2] - v * np.cos(theta) * model.dt) < 1e-10  # df_py/dtheta * dt
    assert abs(B_d[2, 0] - (v / (model.L * np.cos(delta)**2)) * model.dt) < 1e-10
    assert abs(B_d[3, 1] - model.dt) < 1e-10


def test_front_axle_pos():
    model = BicycleModel(L=0.33)
    x = np.array([0.0, 0.0, 0.0, 2.0])
    fa = model.front_axle_pos(x)
    assert abs(fa[0] - 0.33) < 1e-9
    assert abs(fa[1] - 0.0) < 1e-9
