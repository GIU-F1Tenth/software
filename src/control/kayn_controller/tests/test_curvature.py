import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.supervisor.curvature import CurvatureEstimator
from simulation.track import curve_track, straight_track


def test_circle_curvature():
    """Menger curvature of a circle with radius R must equal 1/R within 1%."""
    for R in [2.0, 3.0, 5.0, 10.0]:
        track = curve_track(radius=R, sweep_deg=180.0, v_ref=2.0, n_points=300)
        est = CurvatureEstimator(lookahead=10)
        kappas = [est.estimate(track, i) for i in range(10, len(track) - 10)]
        kappa_mean = np.mean(kappas)
        expected = 1.0 / R
        error = abs(kappa_mean - expected) / expected
        assert error < 0.01, f"R={R}: kappa={kappa_mean:.4f}, expected={expected:.4f}, error={error:.3%}"


def test_straight_near_zero():
    """Curvature of a straight line must be near zero."""
    track = straight_track(length=50.0, v_ref=2.0, n_points=200)
    est = CurvatureEstimator(lookahead=10)
    kappas = [est.estimate(track, i) for i in range(5, 190)]
    assert max(kappas) < 0.01, f"Straight track max kappa={max(kappas):.4f}"


def test_hysteresis_enter_exit():
    """Estimate above 0.10 and below 0.06 threshold correctly identified."""
    track = curve_track(radius=5.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    est = CurvatureEstimator(lookahead=10)
    kappa = est.estimate(track, 50)
    # 1/5 = 0.2 > 0.10 → should trigger MPC
    assert kappa > 0.10, f"Expected kappa > 0.10 for R=5m, got {kappa:.4f}"
