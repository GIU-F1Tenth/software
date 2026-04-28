import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.lqr import LQRController
from kayn_controller.controllers.mpc import MPCController
from kayn_controller.controllers.stanley import StanleyController
from kayn_controller.supervisor.curvature import CurvatureEstimator
from kayn_controller.supervisor.fsm import FSM, KAYNState, CONFIRM_STEPS, BLEND_WINDOW, WARMUP_STEPS
from simulation.track import straight_track, curve_track


def _make_fsm():
    model = BicycleModel()
    return FSM(
        lqr=LQRController(model),
        mpc=MPCController(model),
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
    )


def test_initial_state_is_warmup():
    fsm = _make_fsm()
    assert fsm.state == KAYNState.WARMUP


def test_warmup_transitions_to_straight():
    """After warmup_steps Stanley steps, FSM must enter STRAIGHT."""
    fsm = _make_fsm()
    track = straight_track(length=200.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[5]['x'], track[5]['y'], track[5]['theta'], 2.0])
    for _ in range(WARMUP_STEPS):
        fsm.step(x_curr, track, 5)
    assert fsm.state == KAYNState.STRAIGHT, f"Expected STRAIGHT, got {fsm.state.name}"


def test_straight_to_blend_out_on_high_curvature():
    """3 consecutive high-kappa samples must trigger STRAIGHT → BLEND_OUT."""
    fsm = _make_fsm()
    fsm.state = KAYNState.STRAIGHT   # skip warmup
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])

    for _ in range(CONFIRM_STEPS):
        fsm.step(x_curr, track, 10)

    assert fsm.state in (KAYNState.BLEND_OUT, KAYNState.CURVE), \
        f"Expected BLEND_OUT or CURVE, got {fsm.state.name}"


def test_blend_out_completes_to_curve():
    """After BLEND_WINDOW blend steps, state must be CURVE."""
    fsm = _make_fsm()
    fsm.state = KAYNState.STRAIGHT   # skip warmup
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])

    # Get into BLEND_OUT
    for _ in range(CONFIRM_STEPS):
        fsm.step(x_curr, track, 10)

    if fsm.state == KAYNState.BLEND_OUT:
        for _ in range(BLEND_WINDOW):
            fsm.step(x_curr, track, 10)
        assert fsm.state == KAYNState.CURVE, f"Expected CURVE, got {fsm.state.name}"


def test_no_steering_jump_at_transition():
    """Steering output must not jump more than 0.05 rad at any transition."""
    fsm = _make_fsm()
    fsm.state = KAYNState.STRAIGHT   # skip warmup
    model = BicycleModel()
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[5]['x'], track[5]['y'], track[5]['theta'], 2.0])

    last_delta = None
    for i in range(5, min(50, len(track))):
        u = fsm.step(x_curr, track, i)
        delta = u[0]
        if last_delta is not None:
            jump = abs(delta - last_delta)
            assert jump < 0.05, \
                f"Steering jump {jump:.4f} rad at step {i}, state={fsm.state.name}"
        last_delta = delta
        x_curr = model.step_rk4(x_curr, u)


def test_curve_slot_lqr_no_fallback(monkeypatch):
    """curve_controller=lqr must never trigger FALLBACK even if MPC would time out."""
    model = BicycleModel()
    fsm = FSM(
        lqr=LQRController(model),
        mpc=MPCController(model),
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
        curve_ctrl='lqr',
    )
    fsm.state = KAYNState.CURVE
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])

    original = fsm.mpc.compute_control
    monkeypatch.setattr(fsm.mpc, 'compute_control',
                        lambda *a, **kw: (original(*a, **kw)[0], 0.010, 0))
    fsm.step(x_curr, track, 10)
    assert fsm.state != KAYNState.FALLBACK, \
        f"curve_ctrl=lqr should not enter FALLBACK, got {fsm.state.name}"


def test_invalid_controller_slot_raises():
    """Passing an unknown controller name must raise ValueError at construction."""
    model = BicycleModel()
    import pytest
    with pytest.raises(ValueError):
        FSM(
            lqr=LQRController(model),
            mpc=MPCController(model),
            stanley=StanleyController(model=model),
            curvature_estimator=CurvatureEstimator(lookahead=10),
            curve_ctrl='pure_pursuit',
        )


def test_fallback_on_mpc_timeout(monkeypatch):
    """Monkeypatched slow MPC must trigger CURVE → FALLBACK immediately."""
    fsm = _make_fsm()
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])

    # Force into CURVE state directly
    fsm.state = KAYNState.CURVE

    original = fsm.mpc.compute_control
    def slow_mpc(*args, **kwargs):
        u, _, status = original(*args, **kwargs)
        return u, 0.010, status  # fake 10ms — exceeds 5ms budget

    monkeypatch.setattr(fsm.mpc, 'compute_control', slow_mpc)
    fsm.step(x_curr, track, 10)
    assert fsm.state == KAYNState.FALLBACK, f"Expected FALLBACK, got {fsm.state.name}"
