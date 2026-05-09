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
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from MPC_karim.mpc_solver import KinematicBicycleMPC


CONTROLLER_NAME = 'MPC_karim'


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


class MPCKarimNode(Node):
    def __init__(self) -> None:
        super().__init__('MPC_karim')

        # Topics
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('drive_topic', f'/{CONTROLLER_NAME}/drive')
        self.declare_parameter('selector_topic', '/control_selector')

        # Path source: read csv_path_pub.yaml to discover the CSV path
        self.declare_parameter('csv_path_pub_yaml', '')

        # MPC
        self.declare_parameter('horizon', 15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('control_frequency', 50.0)

        # Cost weights
        self.declare_parameter('q_x', 10.0)
        self.declare_parameter('q_y', 10.0)
        self.declare_parameter('q_v', 1.0)
        self.declare_parameter('q_theta', 2.0)
        self.declare_parameter('r_a', 0.05)
        self.declare_parameter('r_delta', 0.5)

        # Limits
        self.declare_parameter('a_min', -3.0)
        self.declare_parameter('a_max', 3.0)
        self.declare_parameter('delta_min', -0.4)
        self.declare_parameter('delta_max', 0.4)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 5.0)
        self.declare_parameter('speed_factor', 1.0)

        # Resolve params
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.selector_topic = self.get_parameter('selector_topic').value
        self.csv_path_pub_yaml = self.get_parameter('csv_path_pub_yaml').value
        self.N = int(self.get_parameter('horizon').value)
        self.dt = float(self.get_parameter('dt').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.speed_factor = float(self.get_parameter('speed_factor').value)
        self.v_max = float(self.get_parameter('v_max').value)

        self.path_xy: np.ndarray = np.zeros((0, 2))
        self.path_v: np.ndarray = np.zeros((0,))
        self.path_theta: np.ndarray = np.zeros((0,))
        self._load_path()
        if self.path_xy.shape[0] < 2:
            self.get_logger().error('Path is empty or too short; MPC cannot run.')
            raise RuntimeError('Failed to load reference path.')

        self.mpc = KinematicBicycleMPC(
            N=self.N,
            dt=self.dt,
            wheelbase=self.L,
            q_x=float(self.get_parameter('q_x').value),
            q_y=float(self.get_parameter('q_y').value),
            q_v=float(self.get_parameter('q_v').value),
            q_theta=float(self.get_parameter('q_theta').value),
            r_a=float(self.get_parameter('r_a').value),
            r_delta=float(self.get_parameter('r_delta').value),
            a_min=float(self.get_parameter('a_min').value),
            a_max=float(self.get_parameter('a_max').value),
            delta_min=float(self.get_parameter('delta_min').value),
            delta_max=float(self.get_parameter('delta_max').value),
            v_min=float(self.get_parameter('v_min').value),
            v_max=self.v_max,
        )

        # State
        self.last_state: Optional[np.ndarray] = None
        self.is_active = False

        # Pub/sub
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(
            String, self.selector_topic, self._selector_cb, 10
        )

        self.timer = self.create_timer(1.0 / self.control_frequency, self._control_step)

        self.get_logger().info(
            f'MPC_karim started | path waypoints: {self.path_xy.shape[0]} | '
            f'odom={self.odom_topic} | drive={self.drive_topic}'
        )

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
        v = np.clip(v, 0.0, self.v_max)

        # Heading from segment direction; last point copies its predecessor.
        dx = np.diff(xy[:, 0])
        dy = np.diff(xy[:, 1])
        theta = np.arctan2(dy, dx)
        theta = np.append(theta, theta[-1])

        self.path_xy = xy
        self.path_v = v
        self.path_theta = theta
        self.get_logger().info(
            f'Loaded {xy.shape[0]} waypoints from {csv_file} '
            f'(inverse={inverse}, speed_factor={self.speed_factor}).'
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

    # ------------------------------------------------------------ control loop
    def _control_step(self) -> None:
        if not self.is_active or self.last_state is None:
            return

        x_ref = self._build_reference(self.last_state)
        result = self.mpc.solve(self.last_state, x_ref)
        if result is None:
            self.get_logger().warn('MPC solve failed — sending zero command.')
            self._publish(0.0, 0.0)
            return

        a_cmd, delta_cmd = result
        v_now = self.last_state[2]
        speed_cmd = float(np.clip(v_now + a_cmd * self.dt, 0.0, self.v_max))
        self._publish(speed_cmd, delta_cmd)

    # --------------------------------------------------------- reference build
    def _nearest_index(self, x: float, y: float) -> int:
        d2 = (self.path_xy[:, 0] - x) ** 2 + (self.path_xy[:, 1] - y) ** 2
        return int(np.argmin(d2))

    def _build_reference(self, state: np.ndarray) -> np.ndarray:
        """Pick N+1 waypoints starting from the closest one (with wrap-around)."""
        n = self.path_xy.shape[0]
        i0 = self._nearest_index(state[0], state[1])
        idx = [(i0 + k) % n for k in range(self.N + 1)]

        ref = np.zeros((4, self.N + 1))
        ref[0, :] = self.path_xy[idx, 0]
        ref[1, :] = self.path_xy[idx, 1]
        ref[2, :] = self.path_v[idx]
        theta_ref = self.path_theta[idx].copy()

        # Unwrap so jumps across +/-pi don't blow up the cost.
        theta_ref = np.unwrap(theta_ref)
        # Bring the unwrapped reference close to the current yaw.
        while theta_ref[0] - state[3] > math.pi:
            theta_ref -= 2 * math.pi
        while theta_ref[0] - state[3] < -math.pi:
            theta_ref += 2 * math.pi
        ref[3, :] = theta_ref
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
