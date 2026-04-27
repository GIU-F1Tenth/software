# MPC Derivation

## Optimal Control Problem (OCP)
Minimize tracking cost over horizon N=20 steps:

    min   sum_{k=0}^{N-1} (x_k - x_r_k)^T Q (x_k - x_r_k) + u_k^T R u_k
        + (x_N - x_r_N)^T P_f (x_N - x_r_N)

    subject to:
        x_{k+1} = f(x_k, u_k)      (bicycle ODE, RK4)
        |delta_k| <= 0.41888        (steering limit)
        |a_k|     <= 5.0            (acceleration limit)
        v_k       >= 0              (no reverse)

Where:
    x_k = [px, py, theta, v]
    u_k = [delta, a]
    Q   = diag([q_px, q_py, q_theta, q_v])
    R   = diag([r_delta, r_a])
    P_f = 10*Q  (terminal cost, heavier to prevent horizon-end drift)

## From OCP to QP (what acados does internally)

At each step, acados uses the RTI (Real-Time Iteration) scheme:
1. Linearize f around the current trajectory guess (x_bar, u_bar)
2. This turns the NLP into a QP:

    min   0.5 * z^T H z + g^T z
    s.t.  C z = c   (dynamics, linearized)
          D z <= d   (box constraints)

3. Solve QP with HPIPM (interior-point method)
4. Apply one SQP step: x_bar <- x_bar + alpha * dz
5. Warm-start next timestep from shifted previous solution

## CasADi Symbolic Model (what you write in mpc.py)
    f_expl = vertcat(
        v * cos(theta),
        v * sin(theta),
        v * tan(delta) / L,
        a
    )
This IS the bicycle ODE — no abstraction. acados sees it symbolically
and computes the Jacobians automatically for the RTI linearization.

## Failure Handling
- Solve time > 5ms: FSM transitions to FALLBACK
- acados status != 0 (infeasible): FSM transitions to FALLBACK
- During FALLBACK: acados runs in background (output discarded) to monitor recovery
- Recovery: 3 consecutive background solves with status=0 and time < 5ms
