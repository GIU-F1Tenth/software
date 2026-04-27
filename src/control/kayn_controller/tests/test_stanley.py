import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.stanley import StanleyController
from simulation.track import straight_track


def test_right_of_track_gives_positive_steering():
    """Vehicle to the right of track (y=-0.3, negative e_fa) -> positive delta (steer left toward track)."""
    model = BicycleModel()
    stanley = StanleyController(k=1.0, model=model)
    # Vehicle at y=-0.3 (right of track), track along x-axis
    x_curr = np.array([5.0, -0.3, 0.0, 2.0])
    track = straight_track(length=20.0, v_ref=2.0, n_points=200)
    delta = stanley.compute_control(x_curr, track)
    assert delta > 0.0, f"Expected positive delta for right-side error (steer left toward track), got {delta:.4f}"


def test_left_of_track_gives_negative_steering():
    """Vehicle to the left of track (y=+0.3, positive e_fa) -> negative delta (steer right toward track)."""
    model = BicycleModel()
    stanley = StanleyController(k=1.0, model=model)
    x_curr = np.array([5.0, 0.3, 0.0, 2.0])
    track = straight_track(length=20.0, v_ref=2.0, n_points=200)
    delta = stanley.compute_control(x_curr, track)
    assert delta < 0.0, f"Expected negative delta for left-side error (steer right toward track), got {delta:.4f}"


def test_zero_error_zero_steering():
    """On the track with correct heading -> steering near zero."""
    model = BicycleModel()
    stanley = StanleyController(k=1.0, model=model)
    x_curr = np.array([5.0, 0.0, 0.0, 2.0])
    track = straight_track(length=20.0, v_ref=2.0, n_points=200)
    delta = stanley.compute_control(x_curr, track)
    assert abs(delta) < 0.01, f"Expected ~0 steering on-track, got {delta:.4f}"


def test_cte_convergence():
    """
    Vehicle starts 0.4m off straight track.
    Stanley must reduce CTE to < 0.05m within 5 seconds.
    """
    model = BicycleModel(dt=0.02)
    stanley = StanleyController(k=1.5, model=model)
    x_curr = np.array([0.0, 0.4, 0.1, 2.0])
    track = straight_track(length=100.0, v_ref=2.0, n_points=500)

    steps = int(5.0 / model.dt)
    for _ in range(steps):
        delta = stanley.compute_control(x_curr, track)
        u = np.array([delta, 0.0])
        x_curr = model.step_rk4(x_curr, u)

    assert abs(x_curr[1]) < 0.05, f"Stanley CTE={x_curr[1]:.4f}m did not converge"
