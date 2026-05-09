"""Kinematic-bicycle MPC solver (CasADi / IPOPT)."""

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
        N: int,
        dt: float,
        wheelbase: float,
        q_x: float,
        q_y: float,
        q_v: float,
        q_theta: float,
        r_a: float,
        r_delta: float,
        a_min: float,
        a_max: float,
        delta_min: float,
        delta_max: float,
        v_min: float,
        v_max: float,
    ) -> None:
        self.N = int(N)
        self.dt = float(dt)
        self.L = float(wheelbase)

        opti = ca.Opti()
        X = opti.variable(4, self.N + 1)
        U = opti.variable(2, self.N)
        x0 = opti.parameter(4)
        Xref = opti.parameter(4, self.N + 1)

        Q = ca.diag(ca.DM([q_x, q_y, q_v, q_theta]))
        R = ca.diag(ca.DM([r_a, r_delta]))

        cost = 0
        for k in range(self.N):
            err = X[:, k] - Xref[:, k]
            cost += ca.mtimes([err.T, Q, err])
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])
        err_N = X[:, self.N] - Xref[:, self.N]
        cost += ca.mtimes([err_N.T, Q, err_N])
        opti.minimize(cost)

        opti.subject_to(X[:, 0] == x0)
        for k in range(self.N):
            xk, yk, vk, thk = X[0, k], X[1, k], X[2, k], X[3, k]
            ak, dk = U[0, k], U[1, k]
            xdot = ca.vertcat(
                vk * ca.cos(thk),
                vk * ca.sin(thk),
                ak,
                vk * ca.tan(dk) / self.L,
            )
            opti.subject_to(X[:, k + 1] == X[:, k] + self.dt * xdot)

        opti.subject_to(opti.bounded(a_min, U[0, :], a_max))
        opti.subject_to(opti.bounded(delta_min, U[1, :], delta_max))
        opti.subject_to(opti.bounded(v_min, X[2, :], v_max))

        opti.solver(
            'ipopt',
            {'print_time': False},
            {'print_level': 0, 'sb': 'yes', 'max_iter': 80},
        )

        self._opti = opti
        self._X = X
        self._U = U
        self._x0 = x0
        self._Xref = Xref
        self._U_warm = np.zeros((2, self.N))
        self._X_warm = np.zeros((4, self.N + 1))

    def solve(self, current_state: np.ndarray, x_ref: np.ndarray):
        """current_state: shape (4,), x_ref: shape (4, N+1). Returns (a, delta) or None."""
        self._opti.set_value(self._x0, current_state)
        self._opti.set_value(self._Xref, x_ref)
        self._opti.set_initial(self._U, self._U_warm)
        self._opti.set_initial(self._X, self._X_warm)
        try:
            sol = self._opti.solve()
        except RuntimeError:
            return None
        u = sol.value(self._U)
        x = sol.value(self._X)
        self._U_warm = np.hstack([u[:, 1:], u[:, -1:]])
        self._X_warm = np.hstack([x[:, 1:], x[:, -1:]])
        return float(u[0, 0]), float(u[1, 0])
