#!/usr/bin/env python3
"""MPC_karim ROS 2 node.

Tracks a CSV reference trajectory using a kinematic-bicycle MPC.

Inputs:
    - /car_state/odom (nav_msgs/Odometry): vehicle pose+twist in map frame
    - /control_selector (std_msgs/String):  active controller name
    - csv_path_pub.yaml: source of the CSV path (csv_path, inverse)

Output:
    - /MPC_karim/drive (ackermann_msgs/AckermannDriveStamped)
"""

import csv
import math
import os
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# Solver implementations are imported lazily from _make_solver() below so the
# acados backend (which triggers C codegen at import) is only loaded when
# selected.


CONTROLLER_NAME = 'mpc_karim'


def euler_from_quaternion(quaternion):
    """Convert (x, y, z, w) quaternion to (roll, pitch, yaw) in radians."""
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def smooth_angles(theta: np.ndarray, window: int) -> np.ndarray:
    """Moving-average smoothing on a closed-loop angle sequence (handles wrap)."""
    if window <= 1 or theta.shape[0] < window:
        return theta
    sin_s = np.sin(theta)
    cos_s = np.cos(theta)
    kernel = np.ones(window) / window
    sin_f = np.convolve(np.r_[sin_s[-window:], sin_s, sin_s[:window]], kernel, mode='same')
    cos_f = np.convolve(np.r_[cos_s[-window:], cos_s, cos_s[:window]], kernel, mode='same')
    sin_f = sin_f[window:-window]
    cos_f = cos_f[window:-window]
    return np.arctan2(sin_f, cos_f)


class MPCKarimNode(Node):
    def __init__(self) -> None:
        super().__init__('MPC_karim')

        # Topics
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('drive_topic', f'/{CONTROLLER_NAME}/drive')
        self.declare_parameter('selector_topic', '/control_selector')

        # Path source: read csv_path_pub.yaml to discover the CSV path
        self.declare_parameter('csv_path_pub_yaml', '')

        # MPC. 'horizon' is the legacy single-value param; if set,
        # prediction_horizon and control_horizon default to it. New deployments
        # should set prediction_horizon (Np, how far we simulate) and
        # control_horizon (Nc, how many distinct control moves) separately.
        # Nc < Np = "move blocking": control held constant after Nc-1.
        self.declare_parameter('horizon', 15)
        self.declare_parameter('prediction_horizon', 0)  # 0 → fall back to 'horizon'
        self.declare_parameter('control_horizon', 0)     # 0 → fall back to prediction_horizon
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('control_frequency', 50.0)
        # Soft-constraint penalty weights (L1). Higher = bound is harder.
        self.declare_parameter('slack_v_weight', 1000.0)
        self.declare_parameter('slack_delta_weight', 2000.0)
        # Smooth the per-waypoint CSV speed to remove step-changes that would
        # otherwise feed q_v and drive throttle/steering chatter.
        self.declare_parameter('v_smooth_window', 5)
        # Solver backend: 'casadi' (IPOPT) or 'acados' (SQP-RTI + HPIPM).
        # acados is ~10× faster but requires C codegen at first launch.
        self.declare_parameter('solver_type', 'casadi')
        # Directory for acados-generated C code (cached across runs).
        self.declare_parameter(
            'acados_codegen_dir', '/tmp/mpc_karim_acados'
        )

        # Cost weights
        self.declare_parameter('q_x', 10.0)
        self.declare_parameter('q_y', 10.0)
        self.declare_parameter('q_v', 1.0)
        self.declare_parameter('q_theta', 2.0)
        self.declare_parameter('r_a', 0.05)
        self.declare_parameter('r_delta', 2.0)
        self.declare_parameter('r_da', 0.5)
        self.declare_parameter('r_ddelta', 20.0)

        # Limits
        self.declare_parameter('a_min', -3.0)
        self.declare_parameter('a_max', 3.0)
        self.declare_parameter('delta_min', -0.4)
        self.declare_parameter('delta_max', 0.4)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 5.0)
        self.declare_parameter('speed_factor', 1.0)

        # Reference + smoothing + rate limit
        self.declare_parameter('min_lookahead_speed', 1.0)
        self.declare_parameter('theta_smooth_window', 5)
        self.declare_parameter('steer_rate_limit', 4.0)  # rad/s on published cmd
        # Shifts the first reference sample forward along the path so the MPC
        # plans into the corner instead of toward whatever waypoint is laterally
        # nearest to the car right now.
        self.declare_parameter('reference_lookahead_distance', 0.0)

        # ---- Adaptive (speed/curvature scheduled) weights -------------------
        # The "straight" profile is used at high speed / low curvature: softer
        # position pull and stronger Δδ damping → no oscillation on straights.
        # The "corner" profile is used at low speed / high curvature: stiffer
        # position pull, looser Δδ → the car can actually carve the turn.
        # Active profile is a linear blend driven by a single 0..1 "demand"
        # signal built from current speed and upcoming path curvature.
        self.declare_parameter('adaptive_enable', True)
        # Curvature breakpoints (1/m). Below sweeper_start: pure straight.
        # Between sweeper_start and full_sweeper: blend straight→sweeper.
        # Between full_sweeper and full_corner: blend sweeper→corner.
        # Above full_corner: pure corner.
        self.declare_parameter('curvature_sweeper_start', 0.25)
        self.declare_parameter('curvature_full_sweeper', 0.7)
        self.declare_parameter('curvature_full_corner', 1.5)
        # Integrated-heading-change thresholds (rad). These let a long gentle
        # arc trigger the sweeper/corner profile even when peak kappa is low,
        # and also catch *short* sharp kinks via the dtheta channel.
        self.declare_parameter('dtheta_sweeper_start', 0.20)   # ~11°
        self.declare_parameter('dtheta_full_sweeper', 0.60)    # ~34°
        self.declare_parameter('dtheta_full_corner', 1.20)     # ~69°
        # Speed (m/s) at which scheduling fully switches to the straight side.
        self.declare_parameter('speed_full_straight', 3.5)
        # How many horizon steps ahead to inspect for upcoming curvature.
        self.declare_parameter('curvature_lookahead_steps', 10)
        # After alpha rises, hold it for a window before relaxing. Prevents
        # back-to-back corners separated by a brief straight from dropping
        # back to the straight profile mid-chicane. Value in [0,1) per tick:
        # 0.0 disables (instant drop), 0.95 ≈ 1 s memory at 50 Hz.
        self.declare_parameter('alpha_hold_decay', 0.92)
        # Straight-line profile (high-speed, low curvature).
        self.declare_parameter('q_x_straight', 4.0)
        self.declare_parameter('q_y_straight', 4.0)
        self.declare_parameter('q_theta_straight', 6.0)
        self.declare_parameter('r_delta_straight', 2.0)
        self.declare_parameter('r_ddelta_straight', 120.0)
        self.declare_parameter('lookahead_straight', 1.0)
        # Sweeper profile (long gentle arcs). Stiffer than straight, looser
        # rate-penalty so the car can hold sustained steering, but not as
        # aggressive as a tight-corner attack.
        self.declare_parameter('q_x_sweeper', 8.0)
        self.declare_parameter('q_y_sweeper', 8.0)
        self.declare_parameter('q_theta_sweeper', 4.0)
        self.declare_parameter('r_delta_sweeper', 1.0)
        self.declare_parameter('r_ddelta_sweeper', 70.0)
        self.declare_parameter('lookahead_sweeper', 0.6)
        # Corner profile (low-speed, high curvature).
        self.declare_parameter('q_x_corner', 10.0)
        self.declare_parameter('q_y_corner', 10.0)
        self.declare_parameter('q_theta_corner', 4.0)
        self.declare_parameter('r_delta_corner', 0.8)
        self.declare_parameter('r_ddelta_corner', 40.0)
        self.declare_parameter('lookahead_corner', 0.3)

        # ---- Curvature-based corner braking --------------------------------
        # The published speed command is also capped by sqrt(a_lat_max/kappa)
        # — the max speed a kinematic bicycle can sustain the upcoming corner
        # at without exceeding a target lateral acceleration. Decouples brake
        # zones from whatever speeds the CSV happens to encode.
        self.declare_parameter('corner_brake_enable', True)
        self.declare_parameter('lateral_accel_max', 3.0)   # m/s^2
        self.declare_parameter('corner_brake_min_speed', 0.8)  # m/s floor
        self.declare_parameter('corner_brake_lookahead_steps', 25)

        # Resolve params
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.selector_topic = self.get_parameter('selector_topic').value
        self.csv_path_pub_yaml = self.get_parameter('csv_path_pub_yaml').value
        # Resolve horizons. prediction_horizon=0/control_horizon=0 means
        # "fall back to the legacy single 'horizon' value" so existing
        # configs keep working unchanged.
        legacy_horizon = int(self.get_parameter('horizon').value)
        np_param = int(self.get_parameter('prediction_horizon').value)
        nc_param = int(self.get_parameter('control_horizon').value)
        self.Np = np_param if np_param > 0 else legacy_horizon
        self.Nc = nc_param if nc_param > 0 else self.Np
        self.Nc = max(1, min(self.Nc, self.Np))
        self.N = self.Np  # backward-compat alias used in reference builder
        self.v_smooth_window = int(self.get_parameter('v_smooth_window').value)
        self.dt = float(self.get_parameter('dt').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.speed_factor = float(self.get_parameter('speed_factor').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.min_lookahead_speed = float(self.get_parameter('min_lookahead_speed').value)
        self.theta_smooth_window = int(self.get_parameter('theta_smooth_window').value)
        self.steer_rate_limit = float(self.get_parameter('steer_rate_limit').value)
        self.reference_lookahead_distance = float(
            self.get_parameter('reference_lookahead_distance').value
        )

        # Adaptive scheduling config (read once; weights/lookahead recomputed
        # every solve from current speed and upcoming curvature).
        self.adaptive_enable = bool(self.get_parameter('adaptive_enable').value)
        self.kappa_sweeper_start = float(
            self.get_parameter('curvature_sweeper_start').value
        )
        self.kappa_full_sweeper = float(
            self.get_parameter('curvature_full_sweeper').value
        )
        self.kappa_full_corner = float(self.get_parameter('curvature_full_corner').value)
        self.dtheta_sweeper_start = float(
            self.get_parameter('dtheta_sweeper_start').value
        )
        self.dtheta_full_sweeper = float(
            self.get_parameter('dtheta_full_sweeper').value
        )
        self.dtheta_full_corner = float(
            self.get_parameter('dtheta_full_corner').value
        )
        # Enforce monotonic thresholds: start < full_sweeper < full_corner.
        if not (self.kappa_sweeper_start
                < self.kappa_full_sweeper
                < self.kappa_full_corner):
            self.get_logger().warn(
                'Adaptive curvature thresholds not monotonic; '
                'falling back to defaults 0.25 < 0.7 < 1.5.'
            )
            self.kappa_sweeper_start = 0.25
            self.kappa_full_sweeper = 0.7
            self.kappa_full_corner = 1.5
        self.speed_full_straight = float(self.get_parameter('speed_full_straight').value)
        self.curv_lookahead_steps = int(
            self.get_parameter('curvature_lookahead_steps').value
        )
        self.alpha_hold_decay = float(
            np.clip(self.get_parameter('alpha_hold_decay').value, 0.0, 0.999)
        )
        self.corner_brake_enable = bool(
            self.get_parameter('corner_brake_enable').value
        )
        self.a_lat_max = float(self.get_parameter('lateral_accel_max').value)
        self.v_corner_floor = float(
            self.get_parameter('corner_brake_min_speed').value
        )
        self.brake_lookahead_steps = int(
            self.get_parameter('corner_brake_lookahead_steps').value
        )
        # Baseline values (fall back to legacy 'fixed' weights if adaptive is off).
        self._q_v_fixed = float(self.get_parameter('q_v').value)
        self._r_a_fixed = float(self.get_parameter('r_a').value)
        self._r_da_fixed = float(self.get_parameter('r_da').value)
        self._fixed_weights = (
            float(self.get_parameter('q_x').value),
            float(self.get_parameter('q_y').value),
            float(self.get_parameter('q_theta').value),
            float(self.get_parameter('r_delta').value),
            float(self.get_parameter('r_ddelta').value),
            self.reference_lookahead_distance,
        )
        self._straight_profile = (
            float(self.get_parameter('q_x_straight').value),
            float(self.get_parameter('q_y_straight').value),
            float(self.get_parameter('q_theta_straight').value),
            float(self.get_parameter('r_delta_straight').value),
            float(self.get_parameter('r_ddelta_straight').value),
            float(self.get_parameter('lookahead_straight').value),
        )
        self._sweeper_profile = (
            float(self.get_parameter('q_x_sweeper').value),
            float(self.get_parameter('q_y_sweeper').value),
            float(self.get_parameter('q_theta_sweeper').value),
            float(self.get_parameter('r_delta_sweeper').value),
            float(self.get_parameter('r_ddelta_sweeper').value),
            float(self.get_parameter('lookahead_sweeper').value),
        )
        self._corner_profile = (
            float(self.get_parameter('q_x_corner').value),
            float(self.get_parameter('q_y_corner').value),
            float(self.get_parameter('q_theta_corner').value),
            float(self.get_parameter('r_delta_corner').value),
            float(self.get_parameter('r_ddelta_corner').value),
            float(self.get_parameter('lookahead_corner').value),
        )

        self.path_xy: np.ndarray = np.zeros((0, 2))
        self.path_v: np.ndarray = np.zeros((0,))
        self.path_theta: np.ndarray = np.zeros((0,))
        self.path_s: np.ndarray = np.zeros((0,))  # cumulative arc length
        self.path_kappa: np.ndarray = np.zeros((0,))  # |dtheta/ds| per waypoint
        self._load_path()
        if self.path_xy.shape[0] < 2:
            self.get_logger().error('Path is empty or too short; MPC cannot run.')
            raise RuntimeError('Failed to load reference path.')

        self.solver_type = str(self.get_parameter('solver_type').value).lower()
        self.mpc = self._build_solver()
        self.delta_min = float(self.get_parameter('delta_min').value)
        self.delta_max = float(self.get_parameter('delta_max').value)

        # State
        self.last_state: Optional[np.ndarray] = None
        self.is_active = False
        self.last_steer_cmd: float = 0.0
        self._solve_ms_ema: float = 0.0
        self._solve_log_counter: int = 0
        # Hold the highest recent alpha for a short window so that back-to-back
        # corners don't drop the controller back into the straight profile
        # during the brief low-curvature gap between them.
        self._alpha_hold: float = 0.0

        # Pub/sub
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )

        odom_qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                                depth=10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, odom_qos_profile)
        self.create_subscription(
            String, self.selector_topic, self._selector_cb, 10
        )

        self.timer = self.create_timer(1.0 / self.control_frequency, self._control_step)

        self.get_logger().info(
            f'MPC_karim started | solver={self.solver_type} | '
            f'path waypoints: {self.path_xy.shape[0]} | '
            f'Np={self.Np} Nc={self.Nc} | '
            f'odom={self.odom_topic} | drive={self.drive_topic}'
        )

    # ------------------------------------------------------------ solver init
    def _build_solver(self):
        """Construct the chosen MPC backend (CasADi/IPOPT or acados/HPIPM)."""
        common_kwargs = dict(
            Np=self.Np,
            Nc=self.Nc,
            dt=self.dt,
            wheelbase=self.L,
            q_x=float(self.get_parameter('q_x').value),
            q_y=float(self.get_parameter('q_y').value),
            q_v=float(self.get_parameter('q_v').value),
            q_theta=float(self.get_parameter('q_theta').value),
            r_a=float(self.get_parameter('r_a').value),
            r_delta=float(self.get_parameter('r_delta').value),
            r_da=float(self.get_parameter('r_da').value),
            r_ddelta=float(self.get_parameter('r_ddelta').value),
            a_min=float(self.get_parameter('a_min').value),
            a_max=float(self.get_parameter('a_max').value),
            delta_min=float(self.get_parameter('delta_min').value),
            delta_max=float(self.get_parameter('delta_max').value),
            v_min=float(self.get_parameter('v_min').value),
            v_max=self.v_max,
            slack_v_weight=float(self.get_parameter('slack_v_weight').value),
            slack_delta_weight=float(self.get_parameter('slack_delta_weight').value),
        )
        if self.solver_type == 'acados':
            from MPC_karim.mpc_solver_acados import AcadosKinematicBicycleMPC
            codegen = str(self.get_parameter('acados_codegen_dir').value)
            self.get_logger().info(
                f'Initializing acados solver (codegen dir: {codegen}). '
                f'First run compiles C code; subsequent runs use the cache.'
            )
            return AcadosKinematicBicycleMPC(codegen_dir=codegen, **common_kwargs)
        if self.solver_type != 'casadi':
            self.get_logger().warn(
                f"Unknown solver_type='{self.solver_type}'; falling back to 'casadi'."
            )
        from MPC_karim.mpc_solver import KinematicBicycleMPC
        return KinematicBicycleMPC(**common_kwargs)

    # ------------------------------------------------------------ path loading
    def _load_path(self) -> None:
        yaml_path = self.csv_path_pub_yaml
        if not yaml_path:
            self.get_logger().error("'csv_path_pub_yaml' param is empty.")
            return
        if not os.path.isfile(yaml_path):
            self.get_logger().error(f'csv_path_pub.yaml not found: {yaml_path}')
            return

        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        try:
            params = cfg['csv_path_pub']['ros__parameters']
        except KeyError:
            self.get_logger().error(
                "csv_path_pub.yaml missing 'csv_path_pub.ros__parameters'."
            )
            return

        csv_file = params.get('csv_path', '')
        inverse = bool(params.get('inverse', False))

        if not os.path.isfile(csv_file):
            self.get_logger().error(f'CSV path file not found: {csv_file}')
            return

        rows: List[Tuple[float, float, float]] = []
        with open(csv_file, newline='') as fh:
            for row in csv.reader(fh):
                if not row:
                    continue
                rows.append((float(row[0]), float(row[1]), float(row[2])))

        if inverse:
            rows.reverse()

        xy = np.array([(r[0], r[1]) for r in rows], dtype=float)
        v = np.array([r[2] for r in rows], dtype=float) * self.speed_factor
        # Smooth CSV speed before clipping. Removes step-changes between
        # adjacent waypoints that would otherwise drive throttle (and through
        # coupling, steering) oscillation as the reference v jumps.
        if self.v_smooth_window > 1 and v.shape[0] >= self.v_smooth_window:
            kernel = np.ones(self.v_smooth_window) / float(self.v_smooth_window)
            v = np.convolve(v, kernel, mode='same')
        v = np.clip(v, 0.0, self.v_max)

        # Heading from segment direction; last point copies its predecessor.
        dx = np.diff(xy[:, 0])
        dy = np.diff(xy[:, 1])
        theta = np.arctan2(dy, dx)
        theta = np.append(theta, theta[-1])
        theta = smooth_angles(theta, self.theta_smooth_window)

        # Cumulative arc length along the path (used for distance-based lookahead).
        seg = np.hypot(dx, dy)
        s = np.concatenate([[0.0], np.cumsum(seg)])

        # Per-waypoint curvature |dtheta/ds|. Used by the adaptive scheduler to
        # raise tracking weights and shorten lookahead when a turn is upcoming.
        dtheta = np.diff(np.unwrap(theta))
        ds = np.diff(s)
        ds_safe = np.where(np.abs(ds) < 1e-6, 1e-6, ds)
        kappa = np.abs(dtheta / ds_safe)
        kappa = np.append(kappa, kappa[-1] if kappa.size else 0.0)
        # Light smoothing so a single noisy waypoint doesn't flip the profile.
        if kappa.shape[0] >= 5:
            kernel = np.ones(5) / 5.0
            kappa = np.convolve(kappa, kernel, mode='same')

        self.path_xy = xy
        self.path_v = v
        self.path_theta = theta
        self.path_s = s
        self.path_kappa = kappa
        self.get_logger().info(
            f'Loaded {xy.shape[0]} waypoints from {csv_file} '
            f'(inverse={inverse}, speed_factor={self.speed_factor}, '
            f'total_length={s[-1]:.2f}m).'
        )

    # ----------------------------------------------------------- ROS callbacks
    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.last_state = np.array([x, y, v, yaw], dtype=float)

    def _selector_cb(self, msg: String) -> None:
        was_active = self.is_active
        self.is_active = (msg.data.strip() == CONTROLLER_NAME)
        if self.is_active and not was_active:
            self.get_logger().info('MPC_karim: activated by /control_selector.')
        elif (not self.is_active) and was_active:
            self.get_logger().info('MPC_karim: paused by /control_selector.')
            self.last_steer_cmd = 0.0

    # --------------------------------------------------------- adaptive sched.
    def _scheduled_profile(self, state: np.ndarray) -> Tuple[float, float, float, float, float, float, float]:
        """Return (q_x, q_y, q_theta, r_delta, r_ddelta, lookahead, alpha).

        Three-way blend across straight / sweeper / corner profiles, driven by
        the peak curvature in a lookahead window. Speed demand can pull us up
        to the sweeper profile (not all the way to corner), so slowing down on
        a straight doesn't fake a tight-turn response.

        alpha in [0, 1] is a continuous "demand" indicator:
            0.0       → full straight
            ~0.5      → full sweeper
            1.0       → full corner
        """
        if not self.adaptive_enable or self.path_kappa.shape[0] == 0:
            qx, qy, qth, rd, rdd, look = self._fixed_weights
            return qx, qy, qth, rd, rdd, look, 0.0

        # ---- Curvature demand --------------------------------------------
        # Use *both* peak kappa and total heading change ∫|kappa| ds over the
        # lookahead window. A short-but-sharp kink has high peak; a long
        # gentle sweeper has high integral; a short gentle bend has low both
        # → stays near straight. Pick whichever signal is more aggressive.
        i0 = self._nearest_index(state[0], state[1])
        n = self.path_kappa.shape[0]
        end = min(n, i0 + max(1, self.curv_lookahead_steps))
        if end > i0:
            kappa_window = self.path_kappa[i0:end]
            ds_window = np.diff(self.path_s[i0:end + 1]) \
                if end < n else np.diff(self.path_s[i0:end])
            kappa_ahead = float(np.max(kappa_window))
            # Total heading change (rad) covered by the upcoming window.
            # Long sweepers integrate to large totals; short kinks integrate
            # to small totals even if kappa peaks high.
            if ds_window.size == kappa_window.size:
                total_dtheta = float(np.sum(np.abs(kappa_window) * ds_window))
            else:
                m = min(ds_window.size, kappa_window.size)
                total_dtheta = float(np.sum(np.abs(kappa_window[:m]) * ds_window[:m]))
        else:
            kappa_ahead = 0.0
            total_dtheta = 0.0

        # Map peak kappa onto the three regimes (as before).
        k0 = self.kappa_sweeper_start
        k1 = self.kappa_full_sweeper
        k2 = self.kappa_full_corner
        if kappa_ahead <= k0:
            t_peak = 0.0
        elif kappa_ahead <= k1:
            t_peak = 0.5 * (kappa_ahead - k0) / max(1e-6, k1 - k0)
        elif kappa_ahead <= k2:
            t_peak = 0.5 + 0.5 * (kappa_ahead - k1) / max(1e-6, k2 - k1)
        else:
            t_peak = 1.0

        # Map total heading change onto a similar 0..1 ramp. The thresholds
        # below correspond to "how much the path bends across the window
        # before we commit to sweeper/corner profile."
        d0 = self.dtheta_sweeper_start
        d1 = self.dtheta_full_sweeper
        d2 = self.dtheta_full_corner
        if total_dtheta <= d0:
            t_arc = 0.0
        elif total_dtheta <= d1:
            t_arc = 0.5 * (total_dtheta - d0) / max(1e-6, d1 - d0)
        elif total_dtheta <= d2:
            t_arc = 0.5 + 0.5 * (total_dtheta - d1) / max(1e-6, d2 - d1)
        else:
            t_arc = 1.0

        t_curv = max(t_peak, t_arc)

        # ---- Speed demand: slow → at most sweeper (cap at 0.5). ------------
        v_now = max(0.0, float(state[2]))
        v_full = max(1e-3, self.speed_full_straight)
        t_speed = float(np.clip((1.0 - v_now / v_full) * 0.5, 0.0, 0.5))

        alpha_raw = max(t_curv, t_speed)

        # Hold-and-decay: alpha snaps up immediately to alpha_raw, but only
        # relaxes downward by (1 - decay) per tick. Keeps the corner profile
        # engaged across the brief straight between linked corners.
        decayed = self._alpha_hold * self.alpha_hold_decay
        alpha = max(alpha_raw, decayed)
        self._alpha_hold = alpha

        # ---- Two-stage blend across the three profiles. --------------------
        s = self._straight_profile
        m = self._sweeper_profile
        c = self._corner_profile
        if alpha <= 0.5:
            w = alpha / 0.5  # 0..1 across straight→sweeper
            a, b = s, m
        else:
            w = (alpha - 0.5) / 0.5  # 0..1 across sweeper→corner
            a, b = m, c
        blend = lambda u, v: (1.0 - w) * u + w * v
        return (
            blend(a[0], b[0]),  # q_x
            blend(a[1], b[1]),  # q_y
            blend(a[2], b[2]),  # q_theta
            blend(a[3], b[3]),  # r_delta
            blend(a[4], b[4]),  # r_ddelta
            blend(a[5], b[5]),  # lookahead
            alpha,
        )

    def _corner_speed_cap(self, state: np.ndarray) -> float:
        """Compute max sustainable speed for upcoming curvature.

        Uses v_cap = sqrt(a_lat_max / kappa_effective) where kappa_effective
        is the worst of:
          - peak |kappa| in the window  (catches sharp short turns)
          - mean |kappa| weighted by arc-length over the window  (catches
            sustained mild bends that a peak-only metric would miss)
        Returns v_max when both are negligible.
        """
        if not self.corner_brake_enable or self.path_kappa.shape[0] == 0:
            return self.v_max
        i0 = self._nearest_index(state[0], state[1])
        n = self.path_kappa.shape[0]
        end = min(n, i0 + max(1, self.brake_lookahead_steps))
        if end <= i0:
            return self.v_max
        kappa_window = self.path_kappa[i0:end]
        kappa_peak = float(np.max(kappa_window))
        if end < n:
            ds_window = np.diff(self.path_s[i0:end + 1])
        else:
            ds_window = np.diff(self.path_s[i0:end])
        if ds_window.size > 0:
            m = min(ds_window.size, kappa_window.size)
            arc = float(np.sum(ds_window[:m]))
            if arc > 1e-6:
                kappa_mean = float(np.sum(kappa_window[:m] * ds_window[:m]) / arc)
            else:
                kappa_mean = 0.0
        else:
            kappa_mean = 0.0
        kappa_effective = max(kappa_peak, kappa_mean)
        if kappa_effective < 1e-3:
            return self.v_max
        v_cap = math.sqrt(self.a_lat_max / kappa_effective)
        return float(max(self.v_corner_floor, min(self.v_max, v_cap)))

    # ------------------------------------------------------------ control loop
    def _control_step(self) -> None:
        if not self.is_active or self.last_state is None:
            return

        qx, qy, qth, rd, rdd, look, alpha = self._scheduled_profile(self.last_state)
        # Push scheduled weights into the solver and use the scheduled lookahead
        # when building the reference trajectory for this tick.
        self.mpc.set_weights(
            q_x=qx, q_y=qy, q_v=self._q_v_fixed, q_theta=qth,
            r_a=self._r_a_fixed, r_delta=rd,
            r_da=self._r_da_fixed, r_ddelta=rdd,
        )
        self._current_lookahead = look

        x_ref = self._build_reference(self.last_state)
        t0 = time.perf_counter()
        result = self.mpc.solve(self.last_state, x_ref)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        self._solve_ms_ema = 0.9 * self._solve_ms_ema + 0.1 * solve_ms
        self._solve_log_counter += 1
        if self._solve_log_counter >= int(self.control_frequency * 2):
            self._solve_log_counter = 0
            budget_ms = 1000.0 / self.control_frequency
            v_cap_log = self._corner_speed_cap(self.last_state)
            sv, sd = self.mpc.get_last_slack()
            self.get_logger().info(
                f'MPC solve {self._solve_ms_ema:.1f} ms (budget {budget_ms:.1f} ms) | '
                f'alpha={alpha:.2f} q_xy={qx:.1f} r_ddelta={rdd:.0f} look={look:.2f} '
                f'v_cap={v_cap_log:.2f} | slack v={sv:.2f} d={sd:.3f}'
            )

        if result is None:
            self.get_logger().warn('MPC solve failed — sending zero command.')
            self._publish(0.0, 0.0)
            return

        a_cmd, delta_cmd = result
        v_now = self.last_state[2]
        # Corner-aware speed cap: never publish faster than the upcoming
        # curvature can sustain at our chosen lateral-accel budget.
        v_cap = self._corner_speed_cap(self.last_state)
        speed_cmd = float(np.clip(v_now + a_cmd * self.dt, 0.0, min(self.v_max, v_cap)))

        # Rate-limit steering on the published command.
        max_step = self.steer_rate_limit / self.control_frequency
        delta_cmd = float(np.clip(
            delta_cmd,
            self.last_steer_cmd - max_step,
            self.last_steer_cmd + max_step,
        ))
        delta_cmd = float(np.clip(delta_cmd, self.delta_min, self.delta_max))
        self.last_steer_cmd = delta_cmd

        self._publish(speed_cmd, delta_cmd)

    # --------------------------------------------------------- reference build
    def _nearest_index(self, x: float, y: float) -> int:
        d2 = (self.path_xy[:, 0] - x) ** 2 + (self.path_xy[:, 1] - y) ** 2
        return int(np.argmin(d2))

    def _build_reference(self, state: np.ndarray) -> np.ndarray:
        """Pick N+1 waypoints by arc-length distance ahead of the nearest point.

        The k-th reference is sampled at s_k = s_nearest + max(v, v_min) * k * dt.
        This keeps the lookahead horizon covering the same physical distance the
        car will travel in N*dt, regardless of waypoint spacing.
        """
        n = self.path_xy.shape[0]
        i0 = self._nearest_index(state[0], state[1])
        v_lookahead = max(float(state[2]), self.min_lookahead_speed)

        total_len = float(self.path_s[-1]) + float(
            np.hypot(self.path_xy[0, 0] - self.path_xy[-1, 0],
                     self.path_xy[0, 1] - self.path_xy[-1, 1])
        )
        s0 = float(self.path_s[i0])

        ref = np.zeros((4, self.N + 1))
        theta_seq = np.zeros(self.N + 1)
        lookahead = getattr(self, '_current_lookahead', self.reference_lookahead_distance)
        for k in range(self.N + 1):
            sk = (s0 + lookahead
                  + v_lookahead * k * self.dt) % total_len
            idx = int(np.searchsorted(self.path_s, sk, side='right') - 1)
            idx = max(0, min(idx, n - 1))
            idx_next = (idx + 1) % n
            seg_len = self.path_s[idx_next] - self.path_s[idx] if idx_next > idx \
                else total_len - self.path_s[idx]
            t = 0.0 if seg_len <= 1e-9 else (sk - self.path_s[idx]) / seg_len
            t = float(np.clip(t, 0.0, 1.0))
            ref[0, k] = (1 - t) * self.path_xy[idx, 0] + t * self.path_xy[idx_next, 0]
            ref[1, k] = (1 - t) * self.path_xy[idx, 1] + t * self.path_xy[idx_next, 1]
            ref[2, k] = (1 - t) * self.path_v[idx] + t * self.path_v[idx_next]
            # Interpolate heading via sin/cos to avoid step-jumps at waypoint
            # boundaries (piecewise-constant was driving q_theta error spikes
            # at every waypoint crossing).
            s_th = (1 - t) * math.sin(self.path_theta[idx]) \
                + t * math.sin(self.path_theta[idx_next])
            c_th = (1 - t) * math.cos(self.path_theta[idx]) \
                + t * math.cos(self.path_theta[idx_next])
            theta_seq[k] = math.atan2(s_th, c_th)

        # Unwrap headings so jumps across +/-pi don't blow up the cost,
        # and pull them close to the current yaw.
        theta_seq = np.unwrap(theta_seq)
        while theta_seq[0] - state[3] > math.pi:
            theta_seq -= 2 * math.pi
        while theta_seq[0] - state[3] < -math.pi:
            theta_seq += 2 * math.pi
        ref[3, :] = theta_seq
        return ref

    def _publish(self, speed: float, steer: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCKarimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
