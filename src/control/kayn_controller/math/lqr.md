# LQR Derivation

## Problem
Track a reference trajectory x_r(t) using a linearized bicycle model.

## State Error
e = x - x_r = [e_px, e_py, e_theta, e_v]

## Linearized Error Dynamics
Linearize f(x, u) around (x_r, u_r):
    e_{k+1} = A_d * e_k + B_d * (u_k - u_r)

where A_d = I + A*dt, B_d = B*dt (Euler discretization).

## Infinite-Horizon LQR Cost
J = sum_{k=0}^{inf} [ e_k^T Q e_k + v_k^T R v_k ]

where v_k = u_k - u_r is the feedback correction input.

Q = diag([q_px, q_py, q_theta, q_v])   (state penalty)
R = diag([r_delta, r_a])                (control effort penalty)

## Discrete Algebraic Riccati Equation (DARE)
Solve for P (4x4 symmetric positive definite):
    P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q

Solved via scipy.linalg.solve_discrete_are(A_d, B_d, Q, R).

## Optimal Gain
    K = (R + B_d^T P B_d)^{-1} B_d^T P A_d     shape: (2, 4)

## Control Law
    v* = -K * e
    u* = u_r + v*

## Implementation Notes
- K is cached and only recomputed when ||x_r - x_r_cached|| > 1e-3
- Heading error e[2] is wrapped to [-pi, pi] before applying K
- Output clipped: delta in [-DELTA_MAX, DELTA_MAX], a in [-A_MAX, A_MAX]
