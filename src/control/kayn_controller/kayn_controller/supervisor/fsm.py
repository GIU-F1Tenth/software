"""
KAYN FSM Supervisor
===================

6 states: WARMUP, STRAIGHT, BLEND_OUT, CURVE, BLEND_IN, FALLBACK

Each state has an independently configurable controller slot (kayn_params.yaml):
  fsm.warmup_controller   : controller run during warmup       (default: stanley)
  fsm.straight_controller : controller run on straights        (default: lqr)
  fsm.curve_controller    : controller run on curves           (default: mpc)
  fsm.fallback_controller : controller run when curve fails    (default: stanley)

Valid values: "stanley" | "lqr" | "mpc"

Transition table:
  WARMUP    → STRAIGHT  : WARMUP_STEPS elapsed
  STRAIGHT  → BLEND_OUT : kappa > 0.10 for CONFIRM_STEPS consecutive samples
  BLEND_OUT → CURVE     : BLEND_WINDOW steps complete (blends straight→curve ctrl)
  CURVE     → BLEND_IN  : kappa < 0.06 for CONFIRM_STEPS consecutive samples
  CURVE     → FALLBACK  : curve_ctrl=mpc AND (solve_time > 5ms OR status != 0)
  BLEND_IN  → STRAIGHT  : BLEND_WINDOW steps complete (blends curve→straight ctrl)
  FALLBACK  → CURVE     : curve_ctrl=mpc AND CONFIRM_STEPS healthy MPC solves AND kappa > 0.10
  FALLBACK  → STRAIGHT  : curve_ctrl=mpc AND CONFIRM_STEPS healthy MPC solves AND kappa <= 0.10

Blend zones: linear interpolation over BLEND_WINDOW steps, preventing steering jumps.
"""

import numpy as np
from enum import Enum, auto
from typing import List, Dict

from .curvature import CurvatureEstimator, ENTER_THRESHOLD, EXIT_THRESHOLD
from .state_handoff import handoff

BLEND_WINDOW  = 5       # steps for output blending at transitions
CONFIRM_STEPS = 3       # consecutive samples needed to confirm a transition
MPC_TIMEOUT_S = 0.005   # 5ms solver budget
WARMUP_STEPS  = 50      # warmup steps before handing off to straight controller (1s at 50Hz)

_VALID_CTRLS = {'stanley', 'lqr', 'mpc'}


class KAYNState(Enum):
    WARMUP    = auto()
    STRAIGHT  = auto()
    BLEND_OUT = auto()   # straight → curve transition
    CURVE     = auto()
    BLEND_IN  = auto()   # curve → straight transition
    FALLBACK  = auto()


class FSM:
    def __init__(self, lqr, mpc, stanley, curvature_estimator: CurvatureEstimator,
                 warmup_steps: int = WARMUP_STEPS,
                 warmup_ctrl: str = 'stanley',
                 straight_ctrl: str = 'lqr',
                 curve_ctrl: str = 'mpc',
                 fallback_ctrl: str = 'stanley'):
        for slot, val in [('warmup', warmup_ctrl), ('straight', straight_ctrl),
                          ('curve', curve_ctrl),   ('fallback', fallback_ctrl)]:
            if val not in _VALID_CTRLS:
                raise ValueError(f"fsm.{slot}_controller={val!r} — must be one of {_VALID_CTRLS}")

        self.lqr      = lqr
        self.mpc      = mpc
        self.stanley  = stanley
        self.curv_est = curvature_estimator
        self._warmup_steps   = warmup_steps
        self._warmup_ctrl    = warmup_ctrl
        self._straight_ctrl  = straight_ctrl
        self._curve_ctrl     = curve_ctrl
        self._fallback_ctrl  = fallback_ctrl

        self.state = KAYNState.WARMUP
        self._warmup_count   = 0
        self._confirm_count  = 0
        self._blend_step     = 0
        self._recovery_count = 0

    def step(self, x_curr: np.ndarray, trajectory: List[Dict],
             ref_idx: int) -> np.ndarray:
        """One FSM control step. Returns u = [delta, a]."""
        kappa = self.curv_est.estimate(trajectory, ref_idx)

        if self.state == KAYNState.WARMUP:
            return self._step_warmup(x_curr, trajectory, ref_idx)
        elif self.state == KAYNState.STRAIGHT:
            return self._step_straight(x_curr, trajectory, ref_idx, kappa)
        elif self.state == KAYNState.BLEND_OUT:
            return self._step_blend_out(x_curr, trajectory, ref_idx)
        elif self.state == KAYNState.CURVE:
            return self._step_curve(x_curr, trajectory, ref_idx, kappa)
        elif self.state == KAYNState.BLEND_IN:
            return self._step_blend_in(x_curr, trajectory, ref_idx)
        elif self.state == KAYNState.FALLBACK:
            return self._step_fallback(x_curr, trajectory, ref_idx, kappa)
        return np.zeros(2)

    @property
    def state_name(self) -> str:
        return self.state.name

    # ------------------------------------------------------------------
    # Per-state handlers
    # ------------------------------------------------------------------

    def _step_warmup(self, x_curr, trajectory, ref_idx) -> np.ndarray:
        u, _, _ = self._ctrl_u(self._warmup_ctrl, x_curr, trajectory, ref_idx)
        # Pre-heat straight controller so it's ready immediately after warmup
        try:
            self._ctrl_u(self._straight_ctrl, x_curr, trajectory, ref_idx)
        except Exception:
            pass
        self._warmup_count += 1
        if self._warmup_count >= self._warmup_steps:
            self._transition(KAYNState.STRAIGHT,
                             f"warmup_complete steps={self._warmup_count}",
                             x_curr, trajectory, ref_idx)
        return u

    def _step_straight(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        u, _, _ = self._ctrl_u(self._straight_ctrl, x_curr, trajectory, ref_idx)
        if kappa > ENTER_THRESHOLD:
            self._confirm_count += 1
            if self._confirm_count >= CONFIRM_STEPS:
                self._transition(KAYNState.BLEND_OUT,
                                 f"kappa={kappa:.3f}", x_curr, trajectory, ref_idx)
        else:
            self._confirm_count = 0
        return u

    def _step_blend_out(self, x_curr, trajectory, ref_idx) -> np.ndarray:
        alpha = self._blend_step / BLEND_WINDOW
        u_out, _, _ = self._ctrl_u(self._straight_ctrl, x_curr, trajectory, ref_idx)
        u_in,  _, _ = self._ctrl_u(self._curve_ctrl,    x_curr, trajectory, ref_idx)
        u = (1 - alpha) * u_out + alpha * u_in
        self._blend_step += 1
        if self._blend_step >= BLEND_WINDOW:
            self._transition(KAYNState.CURVE, "blend_complete", x_curr, trajectory, ref_idx)
        return u

    def _step_curve(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        u, solve_time, status = self._ctrl_u(self._curve_ctrl, x_curr, trajectory, ref_idx)

        if self._curve_ctrl == 'mpc' and (status != 0 or solve_time > MPC_TIMEOUT_S):
            reason = (f"solver_timeout={solve_time*1000:.1f}ms"
                      if solve_time > MPC_TIMEOUT_S else f"infeasible status={status}")
            self._transition(KAYNState.FALLBACK, reason, x_curr, trajectory, ref_idx)
            u_fb, _, _ = self._ctrl_u(self._fallback_ctrl, x_curr, trajectory, ref_idx)
            return u_fb

        if kappa < EXIT_THRESHOLD:
            self._confirm_count += 1
            if self._confirm_count >= CONFIRM_STEPS:
                self._transition(KAYNState.BLEND_IN, f"kappa={kappa:.3f}",
                                 x_curr, trajectory, ref_idx)
        else:
            self._confirm_count = 0
        return u

    def _step_blend_in(self, x_curr, trajectory, ref_idx) -> np.ndarray:
        alpha = self._blend_step / BLEND_WINDOW
        u_out, _, _ = self._ctrl_u(self._curve_ctrl,    x_curr, trajectory, ref_idx)
        u_in,  _, _ = self._ctrl_u(self._straight_ctrl, x_curr, trajectory, ref_idx)
        u = (1 - alpha) * u_out + alpha * u_in
        self._blend_step += 1
        if self._blend_step >= BLEND_WINDOW:
            self._transition(KAYNState.STRAIGHT, "blend_complete", x_curr, trajectory, ref_idx)
        return u

    def _step_fallback(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        u, _, _ = self._ctrl_u(self._fallback_ctrl, x_curr, trajectory, ref_idx)

        # Recovery probe only makes sense when the curve controller is MPC
        if self._curve_ctrl == 'mpc':
            try:
                ref_slice = trajectory[ref_idx:ref_idx + self.mpc.N + 1]
                _, bg_time, bg_status = self.mpc.compute_control(x_curr, ref_slice)
                if bg_status == 0 and bg_time < MPC_TIMEOUT_S:
                    self._recovery_count += 1
                else:
                    self._recovery_count = 0
            except Exception:
                self._recovery_count = 0

            if self._recovery_count >= CONFIRM_STEPS:
                target = KAYNState.CURVE if kappa > ENTER_THRESHOLD else KAYNState.STRAIGHT
                self._transition(target, f"solver_recovered kappa={kappa:.3f}",
                                 x_curr, trajectory, ref_idx)
        return u

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _ctrl_u(self, name: str, x_curr: np.ndarray,
                trajectory: List[Dict], ref_idx: int):
        """Run named controller. Returns (u: np.ndarray, solve_time: float, status: int)."""
        if name == 'mpc':
            ref_slice = trajectory[ref_idx:ref_idx + self.mpc.N + 1]
            return self.mpc.compute_control(x_curr, ref_slice)
        elif name == 'lqr':
            u = self.lqr.compute_control(x_curr, self._ref_state(trajectory, ref_idx))
            return u, 0.0, 0
        else:  # stanley
            delta = self.stanley.compute_control(x_curr, trajectory)
            return np.array([delta, 0.0]), 0.0, 0

    def _transition(self, new_state: KAYNState, reason: str,
                    x_curr: np.ndarray, trajectory: List[Dict],
                    ref_idx: int) -> None:
        print(f"[KAYN] {self.state.name} → {new_state.name} | {reason} | idx={ref_idx}")
        self.state = new_state
        self._confirm_count  = 0
        self._blend_step     = 0
        self._recovery_count = 0

        if new_state in (KAYNState.CURVE, KAYNState.BLEND_OUT):
            handoff(self.mpc, x_curr, trajectory, ref_idx)
        elif new_state in (KAYNState.STRAIGHT, KAYNState.BLEND_IN):
            handoff(self.lqr, x_curr, trajectory, ref_idx)

    def _ref_state(self, trajectory: List[Dict], idx: int) -> np.ndarray:
        wp = trajectory[min(idx, len(trajectory) - 1)]
        return np.array([wp['x'], wp['y'], wp['theta'], wp['v']])
