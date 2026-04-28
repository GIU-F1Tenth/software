"""
KAYN closed-loop simulation.
Zero ROS2 imports. Run with: python3 simulation/sim.py
Outputs: simulation/results/kayn_sim.png
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.lqr import LQRController
from kayn_controller.controllers.mpc import MPCController
from kayn_controller.controllers.stanley import StanleyController
from kayn_controller.supervisor.curvature import CurvatureEstimator
from kayn_controller.supervisor.fsm import FSM
from simulation.track import mixed_track


def _closest_idx(x_curr: np.ndarray, trajectory) -> int:
    pts = np.array([[wp['x'], wp['y']] for wp in trajectory])
    dists = np.linalg.norm(pts - x_curr[:2], axis=1)
    return int(np.argmin(dists))


def run_simulation(dt: float = 0.02, v_init: float = 2.0):
    model = BicycleModel(dt=dt)
    fsm = FSM(
        lqr=LQRController(model),
        mpc=MPCController(model, dt=dt),
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
    )

    track = mixed_track()
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], v_init])

    xs, ys, thetas, vs = [], [], [], []
    deltas, accels     = [], []
    ctes, kappas       = [], []
    modes, times       = [], []

    t = 0.0
    max_steps = int(60.0 / dt)

    for _ in range(max_steps):
        ref_idx = _closest_idx(x_curr, track)
        if ref_idx >= len(track) - 1:
            print(f"Reached end of track at t={t:.2f}s")
            break

        u = fsm.step(x_curr, track, ref_idx)

        wp = track[ref_idx]
        # Signed lateral error: perpendicular to track heading
        perp = np.array([-np.sin(wp['theta']), np.cos(wp['theta'])])
        cte = float(np.dot(x_curr[:2] - np.array([wp['x'], wp['y']]), perp))
        kappa = fsm.curv_est.estimate(track, ref_idx)

        xs.append(x_curr[0]);  ys.append(x_curr[1])
        thetas.append(x_curr[2]); vs.append(x_curr[3])
        deltas.append(u[0]);   accels.append(u[1])
        ctes.append(cte);      kappas.append(kappa)
        modes.append(fsm.state_name)
        times.append(t)

        x_curr = model.step_rk4(x_curr, u)
        t += dt

    return {
        'track':  track,
        'x':      np.array(xs),     'y':     np.array(ys),
        'theta':  np.array(thetas), 'v':     np.array(vs),
        'delta':  np.array(deltas), 'accel': np.array(accels),
        'cte':    np.array(ctes),   'kappa': np.array(kappas),
        'mode':   modes,            'time':  np.array(times),
    }


if __name__ == '__main__':
    print("Running KAYN simulation...")
    data = run_simulation()
    from simulation.plot import plot_results
    plot_results(data)
    print("Done. Results in simulation/results/kayn_sim.png")
