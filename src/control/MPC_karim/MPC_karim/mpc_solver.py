"""Kinematic-bicycle MPC solver (CasADi / IPOPT).

Key design choices:
    * Prediction horizon Np and control horizon Nc are decoupled. The input
      is freely optimized over the first Nc steps and held constant from
      step Nc onward ("move blocking"). This shrinks the optimization, smooths
      the solution, and avoids fitting noise on far-horizon steps where the
      kinematic model is least accurate.
    * State bounds are SOFT (slack variables with L1 penalty). Hard state
      bounds cause the solver to bang against limits when the reference is
      momentarily infeasible, producing visible control oscillation. Soft
      bounds let the planner pay a heavy cost to violate, then recover, so
      the optimization stays well conditioned.
    * Input acceleration bounds are kept HARD (real actuator limit). Steering
      bounds are soft so the planner can predict slight overshoot without
      becoming infeasible; the published δ is still clipped by the node.
    * Q, R, Rd diagonals are CasADi parameters so the node can reschedule
      cost weights every solve.
"""

import casadi as ca
import numpy as np


class KinematicBicycleMPC:
    """Single-shot MPC over a kinematic bicycle model.

    State: [x, y, v, theta]    Input: [a, delta]
        xdot     = v*cos(theta)
        ydot     = v*sin(theta)
        vdot     = a
        thetadot = v*tan(delta)/L
    """

    def __init__(
        self,
        Np: int,
        Nc: int,
        dt: float,
        wheelbase: float,
        q_x: float,
        q_y: float,
        q_v: float,
        q_theta: float,
        r_a: float,
        r_delta: float,
        r_da: float,
        r_ddelta: float,
        a_min: float,
        a_max: float,
        delta_min: float,
        delta_max: float,
        v_min: float,
        v_max: float,
        slack_v_weight: float = 1000.0,
        slack_delta_weight: float = 2000.0,
    ) -> None:
        self.Np = max(1, int(Np))
        self.Nc = max(1, min(int(Nc), self.Np))
        self.dt = float(dt)
        self.L = float(wheelbase)

        opti = ca.Opti()
        X = opti.variable(4, self.Np + 1)
        U = opti.variable(2, self.Nc)
        # Slack variables: non-negative, penalized in the cost. Sv softens the
        # v upper bound at every state node; Sd softens the |delta| bound at
        # every control node.
        Sv = opti.variable(self.Np + 1)
        Sd = opti.variable(self.Nc)

        x0 = opti.parameter(4)
        Xref = opti.parameter(4, self.Np + 1)
        u_prev = opti.parameter(2)
        Qdiag = opti.parameter(4)
        Rdiag = opti.parameter(2)
        Rddiag = opti.parameter(2)
        Q = ca.diag(Qdiag)
        R = ca.diag(Rdiag)
        Rd = ca.diag(Rddiag)

        cost = 0
        # State-tracking cost over the full prediction horizon (uses held U
        # in the dynamics for k >= Nc, so far-horizon error feeds back into
        # the last control degree of freedom).
        for k in range(self.Np):
            err = X[:, k] - Xref[:, k]
            cost += ca.mtimes([err.T, Q, err])
        err_N = X[:, self.Np] - Xref[:, self.Np]
        cost += ca.mtimes([err_N.T, Q, err_N])

        # Input magnitude + Δu cost over the control horizon only.
        for k in range(self.Nc):
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])
            du = U[:, k] - (u_prev if k == 0 else U[:, k - 1])
            cost += ca.mtimes([du.T, Rd, du])

        # Soft-constraint penalties (L1 — large coefficient → exact penalty;
        # slack is exactly zero in the interior, non-zero only when needed).
        cost += slack_v_weight * ca.sum1(Sv)
        cost += slack_delta_weight * ca.sum1(Sd)
        opti.minimize(cost)

        # Dynamics. After step Nc-1 the control is held constant ("move
        # blocking"): u_applied(k) = U[:, min(k, Nc-1)].
        opti.subject_to(X[:, 0] == x0)
        for k in range(self.Np):
            u_idx = min(k, self.Nc - 1)
            xk, yk, vk, thk = X[0, k], X[1, k], X[2, k], X[3, k]
            ak, dk = U[0, u_idx], U[1, u_idx]
            xdot = ca.vertcat(
                vk * ca.cos(thk),
                vk * ca.sin(thk),
                ak,
                vk * ca.tan(dk) / self.L,
            )
            opti.subject_to(X[:, k + 1] == X[:, k] + self.dt * xdot)

        # Hard input bound on acceleration (physical actuator limit).
        opti.subject_to(opti.bounded(a_min, U[0, :], a_max))

        # Soft delta bound: |delta| <= delta_max + slack, slack >= 0.
        opti.subject_to(Sd >= 0)
        for k in range(self.Nc):
            opti.subject_to(U[1, k] <= delta_max + Sd[k])
            opti.subject_to(U[1, k] >= delta_min - Sd[k])

        # Soft v upper bound, hard v lower bound (can't roll backwards).
        opti.subject_to(Sv >= 0)
        opti.subject_to(X[2, :] >= v_min)
        for k in range(self.Np + 1):
            opti.subject_to(X[2, k] <= v_max + Sv[k])

        opti.solver(
            'ipopt',
            {'print_time': False},
            {'print_level': 0, 'sb': 'yes', 'max_iter': 80},
        )

        self._opti = opti
        self._X = X
        self._U = U
        self._Sv = Sv
        self._Sd = Sd
        self._x0 = x0
        self._Xref = Xref
        self._u_prev = u_prev
        self._Qdiag = Qdiag
        self._Rdiag = Rdiag
        self._Rddiag = Rddiag
        self._U_warm = np.zeros((2, self.Nc))
        self._X_warm = np.zeros((4, self.Np + 1))
        self._u_prev_val = np.zeros(2)
        self._Qdiag_val = np.array([q_x, q_y, q_v, q_theta], dtype=float)
        self._Rdiag_val = np.array([r_a, r_delta], dtype=float)
        self._Rddiag_val = np.array([r_da, r_ddelta], dtype=float)
        self._last_slack = (0.0, 0.0)

    def set_weights(
        self,
        q_x: float,
        q_y: float,
        q_v: float,
        q_theta: float,
        r_a: float,
        r_delta: float,
        r_da: float,
        r_ddelta: float,
    ) -> None:
        """Update diagonal Q, R, Rd entries used on the next solve."""
        self._Qdiag_val = np.array([q_x, q_y, q_v, q_theta], dtype=float)
        self._Rdiag_val = np.array([r_a, r_delta], dtype=float)
        self._Rddiag_val = np.array([r_da, r_ddelta], dtype=float)

    def get_last_slack(self):
        """Return (max Sv, max Sd) from the most recent solve. For logging."""
        return self._last_slack

    def solve(self, current_state: np.ndarray, x_ref: np.ndarray):
        """current_state: shape (4,), x_ref: shape (4, Np+1). Returns (a, delta) or None."""
        self._opti.set_value(self._x0, current_state)
        self._opti.set_value(self._Xref, x_ref)
        self._opti.set_value(self._u_prev, self._u_prev_val)
        self._opti.set_value(self._Qdiag, self._Qdiag_val)
        self._opti.set_value(self._Rdiag, self._Rdiag_val)
        self._opti.set_value(self._Rddiag, self._Rddiag_val)
        self._opti.set_initial(self._U, self._U_warm)
        self._opti.set_initial(self._X, self._X_warm)
        try:
            sol = self._opti.solve()
        except RuntimeError:
            return None
        u = np.array(sol.value(self._U))
        if u.ndim == 1:
            u = u.reshape(2, 1)
        x = np.array(sol.value(self._X))
        sv = np.array(sol.value(self._Sv)).ravel()
        sd = np.array(sol.value(self._Sd)).ravel()
        # Rolling warm start. Shift left, repeat last column.
        self._U_warm = np.hstack([u[:, 1:], u[:, -1:]])
        self._X_warm = np.hstack([x[:, 1:], x[:, -1:]])
        a0, d0 = float(u[0, 0]), float(u[1, 0])
        self._u_prev_val = np.array([a0, d0])
        # Clamp tiny IPOPT numerical noise so logs read cleanly.
        self._last_slack = (
            max(0.0, float(np.max(sv))) if sv.size else 0.0,
            max(0.0, float(np.max(sd))) if sd.size else 0.0,
        )
        return a0, d0
