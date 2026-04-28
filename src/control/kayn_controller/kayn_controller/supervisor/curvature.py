"""
Curvature estimator using Menger curvature over a lookahead window.

For three points p1, p2, p3 on a curve, the Menger curvature is:
    kappa = 2 * |cross(p2-p1, p3-p1)| / (|p1p2| * |p2p3| * |p1p3|)

This equals 1/R where R is the radius of the circumscribed circle.
The estimator returns the maximum kappa over a sliding 10-point window,
so it responds to the sharpest upcoming turn in the lookahead.

Hysteresis thresholds (from kayn_params.yaml):
    enter MPC  : kappa > 0.10 rad/m   (R < 10m)
    leave MPC  : kappa < 0.06 rad/m   (R > ~16.7m)
"""

import numpy as np
from typing import List, Dict

ENTER_THRESHOLD = 0.10   # rad/m — enter MPC mode (R < 10m)
EXIT_THRESHOLD  = 0.06   # rad/m — leave MPC mode (R > ~16.7m)


def _menger(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    """Menger curvature of the circle through three 2D points."""
    d12 = np.linalg.norm(p2 - p1)
    d23 = np.linalg.norm(p3 - p2)
    d13 = np.linalg.norm(p3 - p1)
    denom = d12 * d23 * d13
    if denom < 1e-9:
        return 0.0
    cross = abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]))
    return 2.0 * cross / denom


class CurvatureEstimator:
    def __init__(self, lookahead: int = 10):
        self.lookahead = lookahead

    def estimate(self, trajectory: List[Dict], current_idx: int) -> float:
        """
        Return maximum Menger curvature over the lookahead window ahead of current_idx.

        Samples three evenly-spaced points from each consecutive triple in
        [current_idx, current_idx + lookahead] and takes the maximum.
        """
        n = len(trajectory)
        end_idx = min(current_idx + self.lookahead, n - 1)

        if end_idx - current_idx < 2:
            return 0.0

        indices = range(current_idx, end_idx + 1)
        pts = np.array([[trajectory[i]['x'], trajectory[i]['y']] for i in indices])

        kappa_max = 0.0
        for i in range(len(pts) - 2):
            k = _menger(pts[i], pts[i + 1], pts[i + 2])
            if k > kappa_max:
                kappa_max = k

        return kappa_max
