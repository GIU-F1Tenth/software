"""
KAYN FSM Supervisor
===================

6 states: WARMUP, STRAIGHT, BLEND_OUT, CURVE, BLEND_IN, FALLBACK

Transition table:
  WARMUP    → STRAIGHT  : WARMUP_STEPS elapsed (default 50 = 1s at 50Hz)
  STRAIGHT  → BLEND_OUT : kappa > 0.10 for CONFIRM_STEPS consecutive samples
  BLEND_OUT → CURVE     : BLEND_WINDOW steps complete
  CURVE     → BLEND_IN  : kappa < 0.06 for CONFIRM_STEPS consecutive samples
  CURVE     → FALLBACK  : MPC solve_time > 5ms OR solver status != 0
  BLEND_IN  → STRAIGHT  : BLEND_WINDOW steps complete
  FALLBACK  → CURVE     : CONFIRM_STEPS consecutive healthy solves AND kappa > 0.10
  FALLBACK  → STRAIGHT  : CONFIRM_STEPS consecutive healthy solves AND kappa <= 0.10

Blend zones: linear interpolation over BLEND_WINDOW steps, preventing steering jumps.
"""

import numpy as np
import time
from enum import Enum, auto
from typing import List, Dict

from .curvature import CurvatureEstimator, ENTER_THRESHOLD, EXIT_THRESHOLD
from .state_handoff import handoff

BLEND_WINDOW  = 5       # steps for output blending at transitions
CONFIRM_STEPS = 3       # consecutive samples needed to confirm a transition
MPC_TIMEOUT_S = 0.005   # 5ms solver budget
WARMUP_STEPS  = 50      # Stanley warm-up steps before handing to LQR (1s at 50Hz)


class KAYNState(Enum):
    WARMUP    = auto()   # initial Stanley warm-up, pre-heats LQR gain
    STRAIGHT  = auto()
    BLEND_OUT = auto()   # LQR → MPC transition
    CURVE     = auto()
    BLEND_IN  = auto()   # MPC → LQR transition
    FALLBACK  = auto()


class FSM:
    def __init__(self, lqr, mpc, stanley, curvature_estimator: CurvatureEstimator,
                 warmup_steps: int = WARMUP_STEPS):
        self.lqr     = lqr
        self.mpc     = mpc
        self.stanley = stanley
        self.curv_est = curvature_estimator
        self._warmup_steps = warmup_steps

        self.state = KAYNState.WARMUP
        self._warmup_count   = 0
        self._confirm_count  = 0
        self._blend_step     = 0
        self._recovery_count = 0

    def step(self, x_curr: np.ndarray, trajectory: List[Dict],
             ref_idx: int) -> np.ndarray:
        """
        One FSM control step.

        Returns:
            u: [delta, a]
        """
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
        u = np.array([self.stanley.compute_control(x_curr, trajectory), 0.0])
        # Pre-heat LQR gain in background so STRAIGHT is ready the instant we arrive
        try:
            self.lqr.compute_control(x_curr, self._ref_state(trajectory, ref_idx))
        except Exception:
            pass
        self._warmup_count += 1
        if self._warmup_count >= self._warmup_steps:
            self._transition(KAYNState.STRAIGHT,
                             f"warmup_complete steps={self._warmup_count}",
                             x_curr, trajectory, ref_idx)
        return u

    def _step_straight(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        u = self.lqr.compute_control(x_curr, self._ref_state(trajectory, ref_idx))

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
        u_lqr = self.lqr.compute_control(x_curr, self._ref_state(trajectory, ref_idx))
        ref_slice = trajectory[ref_idx:ref_idx + self.mpc.N + 1]
        u_mpc, _, _ = self.mpc.compute_control(x_curr, ref_slice)

        u = (1 - alpha) * u_lqr + alpha * u_mpc
        self._blend_step += 1

        if self._blend_step >= BLEND_WINDOW:
            self._transition(KAYNState.CURVE, "blend_complete",
                             x_curr, trajectory, ref_idx)
        return u

    def _step_curve(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        ref_slice = trajectory[ref_idx:ref_idx + self.mpc.N + 1]
        u_mpc, solve_time, status = self.mpc.compute_control(x_curr, ref_slice)

        if status != 0 or solve_time > MPC_TIMEOUT_S:
            reason = (f"solver_timeout={solve_time*1000:.1f}ms"
                      if solve_time > MPC_TIMEOUT_S else f"infeasible status={status}")
            self._transition(KAYNState.FALLBACK, reason, x_curr, trajectory, ref_idx)
            return np.array([self.stanley.compute_control(x_curr, trajectory), 0.0])

        if kappa < EXIT_THRESHOLD:
            self._confirm_count += 1
            if self._confirm_count >= CONFIRM_STEPS:
                self._transition(KAYNState.BLEND_IN, f"kappa={kappa:.3f}",
                                 x_curr, trajectory, ref_idx)
        else:
            self._confirm_count = 0
        return u_mpc

    def _step_blend_in(self, x_curr, trajectory, ref_idx) -> np.ndarray:
        alpha = self._blend_step / BLEND_WINDOW
        ref_slice = trajectory[ref_idx:ref_idx + self.mpc.N + 1]
        u_mpc, _, _ = self.mpc.compute_control(x_curr, ref_slice)
        u_lqr = self.lqr.compute_control(x_curr, self._ref_state(trajectory, ref_idx))

        u = (1 - alpha) * u_mpc + alpha * u_lqr
        self._blend_step += 1

        if self._blend_step >= BLEND_WINDOW:
            self._transition(KAYNState.STRAIGHT, "blend_complete",
                             x_curr, trajectory, ref_idx)
        return u

    def _step_fallback(self, x_curr, trajectory, ref_idx, kappa) -> np.ndarray:
        u = np.array([self.stanley.compute_control(x_curr, trajectory), 0.0])

        # Background MPC probe to check if solver has recovered
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
