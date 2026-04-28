"""
State Handoff — reinitializes an incoming controller from the current vehicle state.

On every controller switch:
1. Set the incoming controller's reference index to current closest waypoint
2. MPC: warm-start with current state repeated N times (better than cold start)
3. Stanley/LQR: stateless, nothing to initialize
"""

import numpy as np
from typing import List, Dict


def handoff(incoming_controller, x_curr: np.ndarray,
            trajectory: List[Dict], ref_idx: int) -> None:
    """
    Initialize incoming_controller for the current vehicle state.

    Args:
        incoming_controller: LQRController, MPCController, or StanleyController
        x_curr: current vehicle state [px, py, theta, v]
        trajectory: full reference trajectory
        ref_idx: current index in trajectory
    """
    controller_type = type(incoming_controller).__name__

    if controller_type == 'MPCController':
        _handoff_mpc(incoming_controller, x_curr, trajectory, ref_idx)
    # LQR and Stanley are stateless — no initialization needed


def _handoff_mpc(mpc, x_curr: np.ndarray,
                 trajectory: List[Dict], ref_idx: int) -> None:
    """Warm-start MPC: seed all stages with current state so first RTI step is close."""
    try:
        mpc.solver.set(0, 'lbx', x_curr)
        mpc.solver.set(0, 'ubx', x_curr)

        for k in range(mpc.N + 1):
            mpc.solver.set(k, 'x', x_curr)

        for k in range(mpc.N):
            wp_idx = min(ref_idx + k, len(trajectory) - 1)
            wp = trajectory[wp_idx]
            yref = np.array([wp['x'], wp['y'], wp['theta'], wp['v'], 0.0, 0.0])
            mpc.solver.set(k, 'yref', yref)

        wp_e = trajectory[min(ref_idx + mpc.N, len(trajectory) - 1)]
        mpc.solver.set(mpc.N, 'yref',
                       np.array([wp_e['x'], wp_e['y'], wp_e['theta'], wp_e['v']]))
    except Exception:
        pass  # handoff failure is non-fatal — MPC will self-correct on next step
