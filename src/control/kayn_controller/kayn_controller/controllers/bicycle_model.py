"""
Kinematic Bicycle Model
=======================

State:  x = [px, py, theta, v]
Input:  u = [delta, a]

Continuous dynamics:
    px_dot    = v * cos(theta)
    py_dot    = v * sin(theta)
    theta_dot = v * tan(delta) / L
    v_dot     = a

Jacobians (for LQR linearization):
    A = df/dx:
        [0, 0, -v*sin(theta),  cos(theta)      ]
        [0, 0,  v*cos(theta),  sin(theta)      ]
        [0, 0,  0,             tan(delta)/L    ]
        [0, 0,  0,             0               ]

    B = df/du:
        [0,                           0]
        [0,                           0]
        [v / (L * cos(delta)^2),      0]
        [0,                           1]

Discretized via forward Euler (accurate for dt <= 0.02s):
    A_d = I + A * dt
    B_d = B * dt
"""

import numpy as np


L_DEFAULT = 0.33       # F1TENTH wheelbase [m]
DELTA_MAX = 0.41888  # max steering angle [rad] = 24 deg exactly
A_MAX = 5.0            # max acceleration magnitude [m/s^2]
V_MAX = 8.0            # max speed [m/s]


class BicycleModel:
    """
    Kinematic bicycle model for F1TENTH.

    IMPORTANT: This class does NOT clip inputs to physical limits.
    Callers (LQR, MPC, Stanley) are responsible for clamping
    delta to [-DELTA_MAX, DELTA_MAX] and a to [-A_MAX, A_MAX]
    before calling step_rk4() or linearize().
    """

    def __init__(self, L: float = L_DEFAULT, dt: float = 0.02):
        self.L = L
        self.dt = dt

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Continuous dynamics xdot = f(x, u)."""
        px, py, theta, v = x
        delta, a = u
        return np.array([
            v * np.cos(theta),
            v * np.sin(theta),
            v * np.tan(delta) / self.L,
            a,
        ])

    def step_rk4(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Integrate one timestep with RK4."""
        dt = self.dt
        k1 = self.f(x, u)
        k2 = self.f(x + dt / 2 * k1, u)
        k3 = self.f(x + dt / 2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def linearize(self, x_ref: np.ndarray, u_ref: np.ndarray):
        """
        Discrete Jacobians around (x_ref, u_ref).
        Returns (A_d, B_d) each as numpy arrays.
        A_d: (4, 4)   B_d: (4, 2)
        """
        px, py, theta, v = x_ref
        delta, a = u_ref

        A = np.array([
            [0, 0, -v * np.sin(theta),  np.cos(theta)],
            [0, 0,  v * np.cos(theta),  np.sin(theta)],
            [0, 0,  0,                  np.tan(delta) / self.L],
            [0, 0,  0,                  0],
        ])

        cos_d = np.cos(delta)
        B = np.array([
            [0,                                0],
            [0,                                0],
            [v / (self.L * cos_d ** 2),        0],
            [0,                                1],
        ])

        A_d = np.eye(4) + A * self.dt
        B_d = B * self.dt
        return A_d, B_d

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def front_axle_pos(self, x: np.ndarray) -> np.ndarray:
        """Return [px_front, py_front] — front axle position used by Stanley."""
        px, py, theta, v = x
        return np.array([
            px + self.L * np.cos(theta),
            py + self.L * np.sin(theta),
        ])
