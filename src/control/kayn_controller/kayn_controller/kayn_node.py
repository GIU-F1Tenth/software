#!/usr/bin/env python3
"""
KAYN Controller — ROS2 Node

Subscribes:
    /odom                                  nav_msgs/Odometry
    /horizon_mapper/reference_trajectory   giu_f1t_interfaces/VehicleStateArray
    /horizon_mapper/path_ready             std_msgs/Bool

Publishes:
    /kayn/drive             ackermann_msgs/AckermannDriveStamped
    /kayn/mode              std_msgs/String
    /kayn/cross_track_error std_msgs/Float32
    /kayn/curvature         std_msgs/Float32
    /kayn/diagnostics       diagnostic_msgs/DiagnosticArray
"""

import rclpy
import numpy as np
import math
import traceback
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from giu_f1t_interfaces.msg import VehicleStateArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    def euler_from_quaternion(q):
        x, y, z, w = q
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return 0.0, 0.0, math.atan2(siny, cosy)

from .controllers.bicycle_model import BicycleModel
from .controllers.lqr import LQRController
from .controllers.mpc import MPCController
from .controllers.stanley import StanleyController
from .supervisor.curvature import CurvatureEstimator
from .supervisor.fsm import FSM


class KAYNNode(Node):
    def __init__(self):
        super().__init__('kayn_controller_node')
        self._declare_params()
        self._load_params()
        self._build_controllers()
        self._init_state()
        self._setup_subs()
        self._setup_pubs()
        self._setup_timers()
        self.get_logger().info(
            f"KAYN ready | {self.control_hz}Hz | "
            f"warmup={self.warmup_ctrl} straight={self.straight_ctrl} "
            f"curve={self.curve_ctrl} fallback={self.fallback_ctrl}"
        )

    def _declare_params(self):
        p = self.declare_parameter
        p('wheelbase', 0.33);         p('dt', 0.02)
        p('lqr.q_px', 5.0);          p('lqr.q_py', 5.0)
        p('lqr.q_theta', 6.0);       p('lqr.q_v', 1.0)
        p('lqr.r_delta', 4.0);       p('lqr.r_a', 0.3)
        p('mpc.horizon_n', 20);      p('mpc.timeout_ms', 5.0)
        p('mpc.q_px', 5.0);          p('mpc.q_py', 5.0)
        p('mpc.q_theta', 6.0);       p('mpc.q_v', 1.0)
        p('mpc.r_delta', 4.0);       p('mpc.r_a', 0.3)
        p('stanley.k', 1.5)
        p('fsm.warmup_controller',   'stanley')
        p('fsm.straight_controller', 'lqr')
        p('fsm.curve_controller',    'mpc')
        p('fsm.fallback_controller', 'stanley')
        p('fsm.warmup_steps', 50)
        p('fsm.enter_threshold', 0.10); p('fsm.exit_threshold', 0.06)
        p('fsm.confirm_steps', 3);   p('fsm.blend_window', 5)
        p('fsm.lookahead', 10)
        p('max_speed', 8.0);         p('max_steering', 0.4189); p('max_accel', 5.0)
        p('control_hz', 50.0)
        p('odom_topic', '/odom')
        p('trajectory_topic', '/horizon_mapper/reference_trajectory')
        p('path_ready_topic', '/horizon_mapper/path_ready')
        p('drive_topic', '/kayn/drive')
        p('debug', False);            p('log_every_n', 25)

    def _load_params(self):
        g = lambda n: self.get_parameter(n).value
        self.wheelbase    = g('wheelbase');    self.dt          = g('dt')
        self.mpc_n        = g('mpc.horizon_n')
        self.max_speed    = g('max_speed');    self.max_steering = g('max_steering')
        self.max_accel    = g('max_accel');    self.control_hz  = g('control_hz')
        self.odom_topic   = g('odom_topic');   self.traj_topic  = g('trajectory_topic')
        self.ready_topic  = g('path_ready_topic'); self.drive_topic = g('drive_topic')
        self.debug        = g('debug');        self.log_every_n = g('log_every_n')

        self.lqr_Q = np.diag([g('lqr.q_px'), g('lqr.q_py'), g('lqr.q_theta'), g('lqr.q_v')])
        self.lqr_R = np.diag([g('lqr.r_delta'), g('lqr.r_a')])
        self.mpc_Q = np.diag([g('mpc.q_px'), g('mpc.q_py'), g('mpc.q_theta'), g('mpc.q_v')])
        self.mpc_R = np.diag([g('mpc.r_delta'), g('mpc.r_a')])
        self.stanley_k      = g('stanley.k')
        self.warmup_ctrl    = g('fsm.warmup_controller')
        self.straight_ctrl  = g('fsm.straight_controller')
        self.curve_ctrl     = g('fsm.curve_controller')
        self.fallback_ctrl  = g('fsm.fallback_controller')
        self.warmup_steps   = g('fsm.warmup_steps')
        self.curv_lookahead = g('fsm.lookahead')

    def _build_controllers(self):
        model = BicycleModel(L=self.wheelbase, dt=self.dt)
        self.fsm = FSM(
            lqr=LQRController(model, Q=self.lqr_Q, R=self.lqr_R),
            mpc=MPCController(model, N=self.mpc_n, Q=self.mpc_Q, R=self.mpc_R),
            stanley=StanleyController(k=self.stanley_k, model=model),
            curvature_estimator=CurvatureEstimator(lookahead=self.curv_lookahead),
            warmup_steps=self.warmup_steps,
            warmup_ctrl=self.warmup_ctrl,
            straight_ctrl=self.straight_ctrl,
            curve_ctrl=self.curve_ctrl,
            fallback_ctrl=self.fallback_ctrl,
        )

    def _init_state(self):
        self.x_curr     = None
        self.trajectory = []
        self.path_ready = False
        self.ref_idx    = 0
        self._iter      = 0
        self._last_block = None

    def _setup_subs(self):
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)
        rel_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(VehicleStateArray, self.traj_topic, self._traj_cb, rel_qos)
        self.create_subscription(Bool, self.ready_topic, self._ready_cb, rel_qos)

    def _setup_pubs(self):
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        self._pub_drive = self.create_publisher(AckermannDriveStamped, self.drive_topic, qos)
        self._pub_mode  = self.create_publisher(String,  '/kayn/mode', qos)
        self._pub_cte   = self.create_publisher(Float32, '/kayn/cross_track_error', qos)
        self._pub_kappa = self.create_publisher(Float32, '/kayn/curvature', qos)
        self._pub_diag  = self.create_publisher(DiagnosticArray, '/kayn/diagnostics', qos)

    def _setup_timers(self):
        self.create_timer(1.0 / self.control_hz, self._control_cb)
        self.create_timer(1.0, self._diag_cb)

    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.x_curr = np.array([p.x, p.y, yaw, math.sqrt(vx*vx + vy*vy)])
        except Exception as e:
            self.get_logger().error(f"odom_cb: {e}")

    def _traj_cb(self, msg: VehicleStateArray):
        self.trajectory = [
            {'x': s.x, 'y': s.y, 'theta': s.theta, 'v': s.v}
            for s in msg.states
        ]

    def _ready_cb(self, msg: Bool):
        prev = self.path_ready
        self.path_ready = msg.data
        if self.path_ready != prev:
            self.get_logger().info(f"Path ready: {self.path_ready}")

    # ------------------------------------------------------------------

    def _control_cb(self):
        self._iter += 1

        if not self.path_ready:
            self._block("path not ready"); return
        if self.x_curr is None:
            self._block("no odometry"); return
        if len(self.trajectory) < 3:
            self._block("trajectory too short"); return

        try:
            pts = np.array([[wp['x'], wp['y']] for wp in self.trajectory])
            self.ref_idx = int(np.argmin(np.linalg.norm(pts - self.x_curr[:2], axis=1)))

            u = self.fsm.step(self.x_curr, self.trajectory, self.ref_idx)
            self._last_block = None

            # Drive command
            drive = AckermannDriveStamped()
            drive.header.stamp = self.get_clock().now().to_msg()
            drive.header.frame_id = 'base_link'
            drive.drive.steering_angle = float(np.clip(u[0], -self.max_steering, self.max_steering))
            drive.drive.acceleration   = float(np.clip(u[1], -self.max_accel, self.max_accel))
            drive.drive.speed = float(np.clip(
                self.trajectory[self.ref_idx]['v'], 0.0, self.max_speed))
            self._pub_drive.publish(drive)

            # Telemetry
            self._pub_mode.publish(String(data=self.fsm.state_name))

            wp = self.trajectory[self.ref_idx]
            perp = np.array([-math.sin(wp['theta']), math.cos(wp['theta'])])
            cte  = float(np.dot(self.x_curr[:2] - np.array([wp['x'], wp['y']]), perp))
            self._pub_cte.publish(Float32(data=cte))

            kappa = float(self.fsm.curv_est.estimate(self.trajectory, self.ref_idx))
            self._pub_kappa.publish(Float32(data=kappa))

            if self.debug or self._iter % self.log_every_n == 0:
                self.get_logger().info(
                    f"[{self._iter}] mode={self.fsm.state_name} "
                    f"v={self.x_curr[3]:.2f} steer={u[0]:.3f} "
                    f"kappa={kappa:.3f} cte={cte:.3f}"
                )

        except Exception as e:
            self.get_logger().error(f"control_cb: {e}")
            if self.debug:
                traceback.print_exc()

    def _block(self, reason: str):
        if reason != self._last_block:
            self.get_logger().warning(f"KAYN blocked: {reason}")
            self._last_block = reason
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = 0.0
        self._pub_drive.publish(drive)

    def _diag_cb(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        s = DiagnosticStatus()
        s.name = 'kayn_controller'; s.hardware_id = 'kayn'
        s.level = DiagnosticStatus.OK; s.message = self.fsm.state_name
        s.values = [
            KeyValue(key='mode',       value=self.fsm.state_name),
            KeyValue(key='path_ready', value=str(self.path_ready)),
            KeyValue(key='traj_len',   value=str(len(self.trajectory))),
            KeyValue(key='iter',       value=str(self._iter)),
        ]
        msg.status.append(s)
        self._pub_diag.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KAYNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
