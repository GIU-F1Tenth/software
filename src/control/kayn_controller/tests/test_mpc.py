import numpy as np
import time
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.mpc import MPCController
from simulation.track import curve_track, straight_track


def test_ocp_setup_no_error():
    """MPC solver must initialize without throwing."""
    model = BicycleModel()
    mpc = MPCController(model)
    assert mpc.solver is not None


def test_feasible_on_curve():
    """MPC must return status=0 (feasible) on a curved reference."""
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)

    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]

    u, solve_time, status = mpc.compute_control(x_curr, ref_traj)
    assert status == 0, f"MPC infeasible (status={status})"
    assert u.shape == (2,)


def test_solve_time_within_budget():
    """acados RTI solve must complete in < 10ms (budget is 5ms, 2x margin)."""
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]

    # Warm up
    mpc.compute_control(x_curr, ref_traj)

    # Measure
    _, solve_time, _ = mpc.compute_control(x_curr, ref_traj)
    assert solve_time < 0.010, f"MPC too slow: {solve_time*1000:.1f}ms"


def test_steering_output_within_limits():
    """Steering output must be within [-0.4189, 0.4189]."""
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]

    u, _, _ = mpc.compute_control(x_curr, ref_traj)
    assert abs(u[0]) <= 0.4189 + 1e-6, f"delta={u[0]:.4f} exceeds limit"


def test_acados_matches_intuition_on_straight():
    """On a straight reference with no lateral error, steering should be near zero."""
    model = BicycleModel()
    mpc = MPCController(model)
    track = straight_track(length=50.0, v_ref=2.0, n_points=200)
    x_curr = np.array([0.0, 0.0, 0.0, 2.0])
    ref_traj = track[:mpc.N + 1]
    u, _, status = mpc.compute_control(x_curr, ref_traj)
    assert status == 0
    assert abs(u[0]) < 0.05, f"Expected near-zero steering on straight, got {u[0]:.4f}"
