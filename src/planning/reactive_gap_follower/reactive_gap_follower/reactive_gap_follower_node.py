"""Thin ROS 2 wrapper around the disparity extender.

Subscribes to a raw LaserScan, runs the planner, publishes:
- AckermannDriveStamped on the drive topic (routed by control_gateway).
- LaserScan of the processed (post-extension) ranges, for RViz debugging.
- Marker showing the chosen goal direction.

The wrapper owns no algorithm logic; everything decision-related lives in
`disparity_extender.py` so it can be unit-tested without ROS.
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker

from reactive_gap_follower.disparity_extender import (
    DisparityConfig,
    DisparityExtender,
)


class ReactiveGapFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("reactive_gap_follower_node")

        # ---- Topic parameters ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/reactive_gap_follower/drive")
        self.declare_parameter(
            "processed_scan_topic", "/reactive_gap_follower/scan_processed"
        )
        self.declare_parameter("goal_marker_topic", "/reactive_gap_follower/goal")
        self.declare_parameter("diagnostics_topic", "/reactive_gap_follower/diagnostics")
        self.declare_parameter("control_selector_topic", "/control_selector")

        # ---- Behavior gates ----
        self.declare_parameter("only_publish_when_selected", True)
        self.declare_parameter("publish_processed_scan", True)
        self.declare_parameter("publish_goal_marker", True)

        # ---- Algorithm parameters (flat layout for plain YAML loading) ----
        self.declare_parameter("car_half_width", 0.15)
        self.declare_parameter("safety_margin", 0.05)
        self.declare_parameter("min_range", 0.10)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("fov_half_deg", 90.0)
        self.declare_parameter("disparity_threshold", 0.30)
        self.declare_parameter("kp_steer", 0.8)
        self.declare_parameter("max_steer", 0.40)
        self.declare_parameter("min_speed", 1.0)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("speed_range_for_max", 6.0)
        self.declare_parameter("safe_stop_speed", 0.0)

        cfg = DisparityConfig(
            car_half_width=self._p("car_half_width"),
            safety_margin=self._p("safety_margin"),
            min_range=self._p("min_range"),
            max_range=self._p("max_range"),
            fov_half=np.deg2rad(self._p("fov_half_deg")),
            disparity_threshold=self._p("disparity_threshold"),
            kp_steer=self._p("kp_steer"),
            max_steer=self._p("max_steer"),
            min_speed=self._p("min_speed"),
            max_speed=self._p("max_speed"),
            speed_range_for_max=self._p("speed_range_for_max"),
            safe_stop_speed=self._p("safe_stop_speed"),
        )
        self.planner = DisparityExtender(cfg)

        self.only_publish_when_selected = bool(self._p("only_publish_when_selected"))
        self.publish_processed = bool(self._p("publish_processed_scan"))
        self.publish_marker = bool(self._p("publish_goal_marker"))
        self.is_selected = False

        # ---- Subscriptions ----
        self.create_subscription(
            LaserScan,
            str(self._p("scan_topic")),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self._p("control_selector_topic")),
            self._on_selector,
            10,
        )

        # ---- Publishers ----
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, str(self._p("drive_topic")), 10
        )
        self.processed_pub = self.create_publisher(
            LaserScan, str(self._p("processed_scan_topic")), 10
        )
        self.marker_pub = self.create_publisher(
            Marker, str(self._p("goal_marker_topic")), 10
        )
        self.diag_pub = self.create_publisher(
            Float64MultiArray, str(self._p("diagnostics_topic")), 10
        )

        self.get_logger().info(
            f"reactive_gap_follower up; "
            f"inflation_radius={self.planner.inflation_radius:.3f}m "
            f"fov=+/-{np.rad2deg(cfg.fov_half):.0f} deg "
            f"disparity_thresh={cfg.disparity_threshold:.2f}m"
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ---- subscriptions ----

    def _on_selector(self, msg: String) -> None:
        self.is_selected = (msg.data == "reactive_gap_follower")

    def _on_scan(self, msg: LaserScan) -> None:
        t0 = time.perf_counter()
        ranges = np.asarray(msg.ranges, dtype=float)
        result = self.planner.plan(ranges, msg.angle_min, msg.angle_increment)
        compute_ms = (time.perf_counter() - t0) * 1e3

        if not result.valid:
            # Publish a safe-stop drive command regardless of selection so the
            # FSM can react. This is the same pattern gap_follower uses.
            self._publish_drive(0.0, self.planner.config.safe_stop_speed, msg.header)
            self._publish_diag(compute_ms, result)
            self.get_logger().warn(
                f"no valid plan: {result.reason}", throttle_duration_sec=1.0
            )
            return

        if not self.only_publish_when_selected or self.is_selected:
            self._publish_drive(result.steering_angle, result.speed, msg.header)

        if self.publish_processed:
            self._publish_processed_scan(msg, result.processed_ranges)

        if self.publish_marker and result.goal_index >= 0:
            self._publish_goal_marker(msg.header.frame_id, result)

        self._publish_diag(compute_ms, result)

    # ---- publishers ----

    def _publish_drive(self, steer: float, speed: float, header) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def _publish_processed_scan(self, src: LaserScan, processed: np.ndarray) -> None:
        out = LaserScan()
        out.header = src.header
        out.angle_min = src.angle_min
        out.angle_max = src.angle_max
        out.angle_increment = src.angle_increment
        out.time_increment = src.time_increment
        out.scan_time = src.scan_time
        out.range_min = src.range_min
        out.range_max = src.range_max
        out.ranges = [float(r) for r in processed]
        self.processed_pub.publish(out)

    def _publish_goal_marker(self, frame_id: str, result) -> None:
        m = Marker()
        m.header.frame_id = frame_id or "laser"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "reactive_gap_follower"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        start = Point()
        start.x, start.y, start.z = 0.0, 0.0, 0.0
        end = Point()
        end.x = float(result.goal_range * np.cos(result.goal_angle))
        end.y = float(result.goal_range * np.sin(result.goal_angle))
        end.z = 0.0
        m.points = [start, end]
        m.scale.x = 0.05
        m.scale.y = 0.10
        m.scale.z = 0.15
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.marker_pub.publish(m)

    def _publish_diag(self, compute_ms: float, result) -> None:
        d = Float64MultiArray()
        d.data = [
            float(compute_ms),
            float(result.goal_angle),
            float(result.goal_range),
            float(result.steering_angle),
            float(result.speed),
            1.0 if result.valid else 0.0,
        ]
        self.diag_pub.publish(d)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveGapFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
