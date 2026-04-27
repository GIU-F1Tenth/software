"""
Stanley Controller — see math/stanley.md for full derivation.

Control law:
    delta = psi_e - arctan(k * e_fa / (v + eps))

    psi_e  = track_heading - vehicle_heading    (heading error)
    e_fa   = signed CTE at front axle           (positive = LEFT of track, LEFT-normal convention)
    k      = cross-track gain
    eps    = 1e-3 (anti-div-by-zero at standstill)

The minus sign is required because e_fa uses the LEFT-normal perp vector.
e_fa > 0 (left of track) requires negative delta (steer right). Using plus sign would diverge.
"""

import numpy as np
from typing import List, Dict
from .bicycle_model import BicycleModel, DELTA_MAX


class StanleyController:
    def __init__(self, k: float = 1.5, model: BicycleModel = None,
                 epsilon: float = 1e-3):
        self.k = k
        self.model = model or BicycleModel()
        self.epsilon = epsilon

    def _find_closest_idx(self, pos: np.ndarray,
                          trajectory: List[Dict]) -> int:
        """Return index of trajectory point closest to pos."""
        pts = np.array([[wp['x'], wp['y']] for wp in trajectory])
        dists = np.linalg.norm(pts - pos, axis=1)
        return int(np.argmin(dists))

    def compute_control(self, x_curr: np.ndarray,
                        trajectory: List[Dict]) -> float:
        """
        Compute Stanley steering angle.

        Args:
            x_curr:     current state [px, py, theta, v]
            trajectory: list of {'x','y','theta','v'} dicts

        Returns:
            delta: steering angle [rad], clipped to [-DELTA_MAX, DELTA_MAX]
        """
        fa = self.model.front_axle_pos(x_curr)
        idx = self._find_closest_idx(fa, trajectory)
        wp = trajectory[idx]

        theta_r = wp['theta']
        v = max(x_curr[3], 0.0)

        # Heading error: track heading minus vehicle heading
        psi_e = self.model.normalize_angle(theta_r - x_curr[2])

        # Cross-track error: signed distance from front axle to track
        # Positive = front axle is to the left of track direction
        p_r = np.array([wp['x'], wp['y']])
        perp = np.array([-np.sin(theta_r), np.cos(theta_r)])  # left-normal
        e_fa = float(np.dot(fa - p_r, perp))

        # Stanley law: psi_e - arctan(k*e_fa/v)
        # Minus sign aligns CTE correction with bicycle model convention
        # where positive delta = left turn (increasing theta).
        # e_fa > 0 (left of track) requires negative delta (right turn) to converge.
        cte_term = np.arctan2(self.k * e_fa, v + self.epsilon)
        delta = psi_e - cte_term

        return float(np.clip(delta, -DELTA_MAX, DELTA_MAX))
