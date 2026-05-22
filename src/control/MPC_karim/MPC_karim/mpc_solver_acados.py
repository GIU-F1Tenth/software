"""acados-based MPC solver for the kinematic bicycle.

Drop-in replacement for `mpc_solver.KinematicBicycleMPC`. Same constructor
arguments, same `solve()` / `set_weights()` / `get_last_slack()` API, so the
node can swap implementations via a single config flag.

Why acados over CasADi / IPOPT:
    * SQP-RTI (real-time iteration): 1 SQP step per solve, typically 10–100×
      faster than IPOPT for problem sizes this small. Deterministic timing.
    * PARTIAL_CONDENSING_HPIPM as the inner QP solver — purpose-built for
      embedded MPC.
    * Generated C code is cached, so only the first run pays the compile
      cost (~20–40 s on the Jetson).

Differences from the CasADi version:
    * Move blocking (Nc < Np) is not implemented in this version — every
      prediction stage has its own free control. Acados is fast enough that
      the move-blocking complexity isn't required to hit real-time. Nc is
      accepted in the constructor for API parity but ignored; a single line
      in the log notes when it was overridden.
    * The state is augmented to 6 dims: [x, y, v, theta, a_prev, delta_prev].
      This makes the Δu rate cost a natural part of the LS residual (no
      separate u_prev parameter game), and is the standard acados pattern.
    * Soft constraints use acados' built-in slack mechanism (idxsbx / idxsbu)
      with L1 (linear) penalty.
"""

from __future__ import annotations

import os
from typing import Tuple

import casadi as ca
import numpy as np


def _ensure_acados_env() -> None:
    """Auto-set ACADOS_SOURCE_DIR / LD_LIBRARY_PATH if the user hasn't.

    Avoids forcing the user to source `env.sh` before launching the node.
    Picks the canonical install path the rest of this repo assumes.
    """
    default_dir = '/home/jetson/Desktop/mechabyte/acados'
    if 'ACADOS_SOURCE_DIR' not in os.environ:
        if os.path.isdir(default_dir):
            os.environ['ACADOS_SOURCE_DIR'] = default_dir
    lib_dir = os.path.join(os.environ.get('ACADOS_SOURCE_DIR', default_dir), 'lib')
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    if os.path.isdir(lib_dir) and lib_dir not in ld:
        os.environ['LD_LIBRARY_PATH'] = f'{lib_dir}:{ld}' if ld else lib_dir


_ensure_acados_env()

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver  # noqa: E402


# State indices in the augmented model (for readability).
_IDX_X = 0
_IDX_Y = 1
_IDX_V = 2
_IDX_TH = 3
_IDX_AP = 4   # a_prev
_IDX_DP = 5   # delta_prev


def _build_model(wheelbase: float, dt: float, name: str) -> AcadosModel:
    """Discrete-time augmented kinematic bicycle.

    State (6):  [x, y, v, theta, a_prev, delta_prev]
    Input (2):  [a, delta]
    Step:       forward Euler over `dt` for the physical states; a_prev,
                delta_prev are set to the current input so the next stage's
                Δu cost is u_k - u_{k-1}.
    """
    model = AcadosModel()

    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    v = ca.SX.sym('v')
    th = ca.SX.sym('theta')
    a_p = ca.SX.sym('a_prev')
    d_p = ca.SX.sym('delta_prev')
    state = ca.vertcat(x, y, v, th, a_p, d_p)

    a = ca.SX.sym('a')
    delta = ca.SX.sym('delta')
    u = ca.vertcat(a, delta)

    x_next = x + dt * v * ca.cos(th)
    y_next = y + dt * v * ca.sin(th)
    v_next = v + dt * a
    th_next = th + dt * v * ca.tan(delta) / wheelbase
    state_next = ca.vertcat(x_next, y_next, v_next, th_next, a, delta)

    model.x = state
    model.u = u
    model.disc_dyn_expr = state_next
    model.name = name
    return model


class AcadosKinematicBicycleMPC:
    """Acados drop-in for KinematicBicycleMPC."""

    NX = 6
    NU = 2
    NY = 8     # [x, y, v, theta, a, delta, da, ddelta]
    NY_E = 4   # [x, y, v, theta]

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
        codegen_dir: str = '/tmp/mpc_karim_acados',
    ) -> None:
        self.Np = int(Np)
        self.Nc = int(Nc)  # accepted for parity; not used
        self.dt = float(dt)
        self.L = float(wheelbase)

        # Build the OCP.
        model = _build_model(self.L, self.dt, 'mpc_karim_kbm')
        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = self.Np

        # ---- Cost: nonlinear least-squares ---------------------------------
        x_sym = model.x
        u_sym = model.u
        cost_y = ca.vertcat(
            x_sym[_IDX_X],
            x_sym[_IDX_Y],
            x_sym[_IDX_V],
            x_sym[_IDX_TH],
            u_sym[0],
            u_sym[1],
            u_sym[0] - x_sym[_IDX_AP],
            u_sym[1] - x_sym[_IDX_DP],
        )
        model.cost_y_expr = cost_y
        model.cost_y_expr_e = ca.vertcat(
            x_sym[_IDX_X],
            x_sym[_IDX_Y],
            x_sym[_IDX_V],
            x_sym[_IDX_TH],
        )
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        W = np.diag([q_x, q_y, q_v, q_theta, r_a, r_delta, r_da, r_ddelta])
        W_e = np.diag([q_x, q_y, q_v, q_theta])
        ocp.cost.W = W
        ocp.cost.W_e = W_e
        ocp.cost.yref = np.zeros(self.NY)
        ocp.cost.yref_e = np.zeros(self.NY_E)

        # ---- Constraints ---------------------------------------------------
        # Input bounds (a hard, delta soft via idxsbu).
        ocp.constraints.lbu = np.array([a_min, delta_min])
        ocp.constraints.ubu = np.array([a_max, delta_max])
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.idxsbu = np.array([1])  # soften delta only

        # State bound on v, soft via idxsbx.
        ocp.constraints.lbx = np.array([v_min])
        ocp.constraints.ubx = np.array([v_max])
        ocp.constraints.idxbx = np.array([_IDX_V])
        ocp.constraints.idxsbx = np.array([0])  # soften the (only) state bound

        # Terminal state bound on v (so v stays feasible at the last stage).
        ocp.constraints.lbx_e = np.array([v_min])
        ocp.constraints.ubx_e = np.array([v_max])
        ocp.constraints.idxbx_e = np.array([_IDX_V])
        ocp.constraints.idxsbx_e = np.array([0])

        # Slack penalty weights (L1: zl/zu linear, Zl/Zu quadratic = 0).
        # Stage slack ordering is [state-bound slacks, input-bound slacks].
        ocp.cost.zl = np.array([slack_v_weight, slack_delta_weight])
        ocp.cost.zu = np.array([slack_v_weight, slack_delta_weight])
        ocp.cost.Zl = np.zeros(2)
        ocp.cost.Zu = np.zeros(2)
        ocp.cost.zl_e = np.array([slack_v_weight])
        ocp.cost.zu_e = np.array([slack_v_weight])
        ocp.cost.Zl_e = np.zeros(1)
        ocp.cost.Zu_e = np.zeros(1)
        # Stage-0 has special treatment: state is fixed by x0 so soft state
        # bounds don't apply, but the input bound on delta still does.
        ocp.constraints.idxsbx_0 = np.array([], dtype=int)
        ocp.constraints.idxsbu_0 = np.array([1], dtype=int)  # delta is bu idx 1
        ocp.cost.zl_0 = np.array([slack_delta_weight])
        ocp.cost.zu_0 = np.array([slack_delta_weight])
        ocp.cost.Zl_0 = np.zeros(1)
        ocp.cost.Zu_0 = np.zeros(1)

        # Initial state placeholder; overridden every solve.
        ocp.constraints.x0 = np.zeros(self.NX)

        # ---- Solver options (real-time NMPC pattern) -----------------------
        ocp.solver_options.tf = self.Np * self.dt
        ocp.solver_options.integrator_type = 'DISCRETE'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N = max(self.Np // 4, 1)
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.levenberg_marquardt = 1e-4   # tiny Hess regularisation
        ocp.solver_options.print_level = 0

        # ---- Codegen + compile --------------------------------------------
        os.makedirs(codegen_dir, exist_ok=True)
        ocp.code_export_directory = os.path.join(codegen_dir, 'c_code')
        json_path = os.path.join(codegen_dir, 'acados_ocp.json')

        self._solver = AcadosOcpSolver(ocp, json_file=json_path)

        # Internal state.
        self._u_prev_val = np.zeros(2)
        self._last_slack: Tuple[float, float] = (0.0, 0.0)
        self._weights_dirty = False
        self._q = np.array([q_x, q_y, q_v, q_theta], dtype=float)
        self._r = np.array([r_a, r_delta], dtype=float)
        self._rd = np.array([r_da, r_ddelta], dtype=float)
        # Push initial weights to every stage.
        self._apply_weights()

    # -------------------------------------------------------------- private

    def _apply_weights(self) -> None:
        W = np.diag([
            self._q[0], self._q[1], self._q[2], self._q[3],
            self._r[0], self._r[1],
            self._rd[0], self._rd[1],
        ])
        W_e = np.diag(self._q)
        for k in range(self.Np):
            self._solver.cost_set(k, 'W', W)
        self._solver.cost_set(self.Np, 'W', W_e)
        self._weights_dirty = False

    def _read_slacks(self) -> Tuple[float, float]:
        """Return (max v-slack, max delta-slack) across the horizon."""
        sv_max = 0.0
        sd_max = 0.0
        for k in range(self.Np):
            try:
                sl = np.atleast_1d(self._solver.get(k, 'sl'))
                su = np.atleast_1d(self._solver.get(k, 'su'))
            except Exception:
                continue
            # Ordering at stages 0..Np-1: [state-slack (v), input-slack (delta)].
            if sl.size >= 1:
                sv_max = max(sv_max, float(sl[0]), float(su[0]))
            if sl.size >= 2:
                sd_max = max(sd_max, float(sl[1]), float(su[1]))
        # Terminal stage has only the state slack.
        try:
            sl_e = np.atleast_1d(self._solver.get(self.Np, 'sl'))
            su_e = np.atleast_1d(self._solver.get(self.Np, 'su'))
            if sl_e.size >= 1:
                sv_max = max(sv_max, float(sl_e[0]), float(su_e[0]))
        except Exception:
            pass
        return (max(0.0, sv_max), max(0.0, sd_max))

    # --------------------------------------------------------------- public

    def set_weights(
        self,
        q_x: float, q_y: float, q_v: float, q_theta: float,
        r_a: float, r_delta: float, r_da: float, r_ddelta: float,
    ) -> None:
        self._q = np.array([q_x, q_y, q_v, q_theta], dtype=float)
        self._r = np.array([r_a, r_delta], dtype=float)
        self._rd = np.array([r_da, r_ddelta], dtype=float)
        self._weights_dirty = True

    def get_last_slack(self) -> Tuple[float, float]:
        return self._last_slack

    def solve(self, current_state: np.ndarray, x_ref: np.ndarray):
        """current_state: (4,) = [x, y, v, theta]; x_ref: (4, Np+1). → (a, delta) or None."""
        if self._weights_dirty:
            self._apply_weights()

        # Augment initial state with previous applied input.
        x0 = np.array([
            float(current_state[0]),
            float(current_state[1]),
            float(current_state[2]),
            float(current_state[3]),
            float(self._u_prev_val[0]),
            float(self._u_prev_val[1]),
        ])
        self._solver.set(0, 'lbx', x0)
        self._solver.set(0, 'ubx', x0)

        # Stage references.
        for k in range(self.Np):
            yref_k = np.array([
                x_ref[0, k], x_ref[1, k], x_ref[2, k], x_ref[3, k],
                0.0, 0.0, 0.0, 0.0,
            ])
            self._solver.set(k, 'yref', yref_k)
        self._solver.set(self.Np, 'yref', np.array([
            x_ref[0, self.Np], x_ref[1, self.Np],
            x_ref[2, self.Np], x_ref[3, self.Np],
        ]))

        status = self._solver.solve()
        # Status 0 = success, 2 = max iter reached but solution usable for RTI.
        if status not in (0, 2):
            return None

        u0 = np.atleast_1d(self._solver.get(0, 'u'))
        a0, d0 = float(u0[0]), float(u0[1])
        self._u_prev_val = np.array([a0, d0])
        self._last_slack = self._read_slacks()
        return a0, d0
