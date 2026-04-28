"""
LQR Controller — see math/lqr.md for full derivation.

Summary:
    Linearize bicycle model at x_ref → (A_d, B_d)
    Solve DARE: P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
    Gain:       K = (R + B^T P B)^{-1} B^T P A     shape: (2, 4)
    Control:    u* = u_ref - K (x - x_ref)
"""

import numpy as np
import scipy.linalg
from .bicycle_model import BicycleModel, DELTA_MAX, A_MAX


class LQRController:
    def __init__(self, model: BicycleModel,
                 Q: np.ndarray = None,
                 R: np.ndarray = None):
        self.model = model
        # Q penalizes [px, py, theta, v] errors
        self.Q = Q if Q is not None else np.diag([5.0, 5.0, 6.0, 1.0])
        # R penalizes [delta, a] effort
        self.R = R if R is not None else np.diag([4.0, 0.3])

        self._cached_K: np.ndarray = None
        self._cached_x_ref: np.ndarray = None
        self._cached_u_ref: np.ndarray = None
        self._cache_tol: float = 1e-3

    def compute_gain(self, x_ref: np.ndarray, u_ref: np.ndarray) -> np.ndarray:
        """Solve DARE and return K (2, 4)."""
        A_d, B_d = self.model.linearize(x_ref, u_ref)
        P = scipy.linalg.solve_discrete_are(A_d, B_d, self.Q, self.R)
        # K = (R + B^T P B)^{-1} B^T P A
        K = np.linalg.solve(self.R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)
        return K

    def _should_recompute(self, x_ref: np.ndarray, u_ref: np.ndarray) -> bool:
        if self._cached_K is None:
            return True
        if np.linalg.norm(x_ref - self._cached_x_ref) > self._cache_tol:
            return True
        if np.linalg.norm(u_ref - self._cached_u_ref) > self._cache_tol:
            return True
        return False

    def compute_control(self, x_curr: np.ndarray, x_ref: np.ndarray,
                        u_ref: np.ndarray = None) -> np.ndarray:
        """
        Compute LQR control input.

        Args:
            x_curr: current state [px, py, theta, v]
            x_ref:  reference state [px, py, theta, v]
            u_ref:  feedforward control [delta, a] (zeros if None)

        Returns:
            u: [delta, a] clipped to physical limits
        """
        if u_ref is None:
            u_ref = np.zeros(2)

        if self._should_recompute(x_ref, u_ref):
            self._cached_K = self.compute_gain(x_ref, u_ref)
            self._cached_x_ref = x_ref.copy()
            self._cached_u_ref = u_ref.copy()

        e = x_curr - x_ref
        # Wrap heading error to [-pi, pi] — critical for numerical stability
        e[2] = self.model.normalize_angle(e[2])

        u = u_ref - self._cached_K @ e
        u[0] = np.clip(u[0], -DELTA_MAX, DELTA_MAX)
        u[1] = np.clip(u[1], -A_MAX, A_MAX)
        return u
