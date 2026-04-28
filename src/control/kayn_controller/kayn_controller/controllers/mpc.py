"""
MPC Controller using acados — see math/mpc.md for full derivation.

OCP:
    min   sum_{k=0}^{N-1} (x_k-x_r_k)^T Q (x_k-x_r_k) + u_k^T R u_k
        + (x_N-x_r_N)^T P_f (x_N-x_r_N)
    s.t.  x_{k+1} = f_rk4(x_k, u_k)
          |delta| <= 0.4189, |a| <= 5.0, v >= 0

Solved via acados RTI (one SQP step per control call, warm-started).
"""

import numpy as np
import time
import os
import tempfile
from typing import List, Dict, Tuple

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import MX, vertcat, cos, sin, tan

from .bicycle_model import BicycleModel, DELTA_MAX, A_MAX


class MPCController:
    def __init__(self, model: BicycleModel,
                 N: int = 20,
                 dt: float = None,
                 Q: np.ndarray = None,
                 R: np.ndarray = None):
        self.model = model
        self.N = N
        self.dt = dt or model.dt

        self.Q = Q if Q is not None else np.diag([5.0, 5.0, 6.0, 1.0])
        self.R = R if R is not None else np.diag([4.0, 0.3])
        self.P_f = 10.0 * self.Q  # terminal cost — heavier to prevent horizon-end drift

        self.solver = self._build_solver()

    def _build_acados_model(self) -> AcadosModel:
        """Define CasADi symbolic bicycle model — the ODE is visible here, no abstraction."""
        acados_model = AcadosModel()
        acados_model.name = 'kayn_bicycle'

        # Symbolic states: [px, py, theta, v]
        px    = MX.sym('px')
        py    = MX.sym('py')
        theta = MX.sym('theta')
        v     = MX.sym('v')
        x     = vertcat(px, py, theta, v)

        # Symbolic controls: [delta, a]
        delta = MX.sym('delta')
        a     = MX.sym('a')
        u     = vertcat(delta, a)

        L = self.model.L

        # Continuous bicycle dynamics — same equations as bicycle_model.py
        # acados uses these symbolically to auto-compute Jacobians for RTI linearization
        f_expl = vertcat(
            v * cos(theta),       # px_dot
            v * sin(theta),       # py_dot
            v * tan(delta) / L,   # theta_dot
            a,                    # v_dot
        )

        acados_model.x = x
        acados_model.u = u
        acados_model.f_expl_expr = f_expl
        return acados_model

    def _build_solver(self) -> AcadosOcpSolver:
        """Build the acados OCP solver with HPIPM QP backend and RTI scheme."""
        ocp = AcadosOcp()
        ocp.model = self._build_acados_model()

        nx = 4  # [px, py, theta, v]
        nu = 2  # [delta, a]

        ocp.solver_options.N_horizon = self.N

        # LINEAR_LS cost: y = [x; u], cost = (y - y_ref)^T W (y - y_ref)
        ocp.cost.cost_type   = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # Selection matrices: y = Vx * x + Vu * u
        Vx = np.zeros((nx + nu, nx))
        Vx[:nx, :] = np.eye(nx)
        Vu = np.zeros((nx + nu, nu))
        Vu[nx:, :] = np.eye(nu)

        ocp.cost.Vx   = Vx
        ocp.cost.Vu   = Vu
        ocp.cost.Vx_e = np.eye(nx)

        # Weight matrices [Q block-diag R] and terminal P_f
        ocp.cost.W   = np.block([[self.Q, np.zeros((4, 2))],
                                  [np.zeros((2, 4)), self.R]])
        ocp.cost.W_e = self.P_f

        # Zero reference (overwritten at each compute_control call)
        ocp.cost.yref   = np.zeros(nx + nu)
        ocp.cost.yref_e = np.zeros(nx)

        # Control bounds: |delta| <= DELTA_MAX, |a| <= A_MAX
        ocp.constraints.lbu    = np.array([-DELTA_MAX, -A_MAX])
        ocp.constraints.ubu    = np.array([ DELTA_MAX,  A_MAX])
        ocp.constraints.idxbu  = np.array([0, 1])

        # State bound: v >= 0 (no reverse)
        ocp.constraints.lbx    = np.array([0.0])
        ocp.constraints.ubx    = np.array([100.0])  # effectively unbounded above
        ocp.constraints.idxbx  = np.array([3])       # index of v in state

        # Initial state constraint (set at each call)
        ocp.constraints.x0 = np.zeros(nx)

        # Solver: RTI = one SQP iteration per call, warm-started from previous solution
        ocp.solver_options.tf              = self.N * self.dt
        ocp.solver_options.integrator_type = 'ERK'        # explicit Runge-Kutta (RK4)
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'   # real-time iteration
        ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'  # interior-point QP
        ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON'
        ocp.solver_options.print_level     = 0

        # Build generated C code in temp dir to keep workspace clean
        build_dir = os.path.join(tempfile.gettempdir(), 'kayn_acados')
        os.makedirs(build_dir, exist_ok=True)
        ocp.code_gen_opts.code_export_directory = build_dir
        ocp.code_gen_opts.json_file = os.path.join(build_dir, 'kayn_ocp.json')

        solver = AcadosOcpSolver(ocp)
        return solver

    def compute_control(self, x_curr: np.ndarray,
                        ref_traj: List[Dict]) -> Tuple[np.ndarray, float, int]:
        """
        Run one RTI step.

        Args:
            x_curr:   current state [px, py, theta, v]
            ref_traj: list of >= N+1 dicts {'x','y','theta','v'}

        Returns:
            (u, solve_time_s, status)
            u:          [delta, a] clipped to physical limits
            solve_time: seconds taken by acados solve()
            status:     0 = feasible, nonzero = infeasible or failure
        """
        # Pin initial state via equality constraints
        self.solver.set(0, 'lbx', x_curr)
        self.solver.set(0, 'ubx', x_curr)

        # Load reference trajectory into every stage cost
        for k in range(self.N):
            wp = ref_traj[min(k, len(ref_traj) - 1)]
            yref = np.array([wp['x'], wp['y'], wp['theta'], wp['v'], 0.0, 0.0])
            self.solver.set(k, 'yref', yref)

        wp_e = ref_traj[min(self.N, len(ref_traj) - 1)]
        self.solver.set(self.N, 'yref',
                        np.array([wp_e['x'], wp_e['y'], wp_e['theta'], wp_e['v']]))

        t0 = time.perf_counter()
        status = self.solver.solve()
        solve_time = time.perf_counter() - t0

        u = self.solver.get(0, 'u')
        u[0] = np.clip(u[0], -DELTA_MAX, DELTA_MAX)
        u[1] = np.clip(u[1], -A_MAX, A_MAX)
        return u, solve_time, status
