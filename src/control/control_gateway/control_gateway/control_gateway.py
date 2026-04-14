#!/usr/bin/env python3

from functools import partial
import importlib
from typing import Dict, Optional, Type

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


from ackermann_msgs.msg import AckermannDriveStamped


class ControlGateway(Node):
    def __init__(self) -> None:
        super().__init__("control_gateway")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("selector_topic", "/control_selector")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("enable_button_index", 4)
        self.declare_parameter("controllers", ["lqr"])
        self.declare_parameter("default_controller", "lqr")
        self.declare_parameter(
            "controller_teleop_output_topic", "/teleop/drive")
        self.declare_parameter(
            "controller_pure_pursuit_output_topic", "/pp/drive")
        self.declare_parameter(
            "controller_gap_following_output_topic", "/gap_following/drive")
        self.declare_parameter("controller_lqr_output_topic", "/lqr/drive")

        self.joy_topic = self.get_parameter("joy_topic").value
        self.selector_topic = self.get_parameter("selector_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.enable_button_index = int(
            self.get_parameter("enable_button_index").value)
        self.controllers = self.get_parameter("controllers").value
        self.default_controller = self.get_parameter(
            "default_controller").value

        if not self.controllers:
            raise ValueError(
                "Parameter 'controllers' is empty. "
                "You must provide at least one controller name."
            )

        self.controller_topics: Dict[str, str] = {}
        all_controller_keys = {param for param in self.get_parameters_by_prefix(
            "").keys() if param.startswith("controller_")}
        for controller_name in self.controllers:
            key = f"controller_{controller_name}_output_topic"

            if key not in all_controller_keys:
                raise ValueError(
                    f"Missing parameter '{key}'"
                )

            topic_name = self.get_parameter(key).value
            if not isinstance(topic_name, str) or not topic_name:
                raise ValueError(
                    f"Invalid topic for controller '{controller_name}': {topic_name}"
                )

            self.controller_topics[controller_name] = topic_name

        self.enabled = False
        self.selected_controller: Optional[str] = None

        if self.default_controller:
            if self.default_controller not in self.controllers:
                raise ValueError(
                    f"default_controller '{self.default_controller}' "
                    f"is not in controllers list: {self.controllers}"
                )
            self.selected_controller = self.default_controller
        else:
            self.selected_controller = self.controllers[0]

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )

        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

        self.selector_sub = self.create_subscription(
            String,
            self.selector_topic,
            self.selector_callback,
            10,
        )

        self.controller_subs = []
        for controller_name, topic_name in self.controller_topics.items():
            sub = self.create_subscription(
                AckermannDriveStamped,
                topic_name,
                partial(self.controller_callback, controller_name),
                10,
            )
            self.controller_subs.append(sub)

        self.get_logger().info("control_gateway started")
        self.get_logger().info(f"  joy_topic: {self.joy_topic}")
        self.get_logger().info(f"  selector_topic: {self.selector_topic}")
        self.get_logger().info(f"  drive_topic: {self.drive_topic}")
        self.get_logger().info(
            f"  enable_button_index: {self.enable_button_index}")
        self.get_logger().info(
            f"  selected_controller: {self.selected_controller}")
        self.get_logger().info(f"  controllers: {self.controllers}")
        for name, topic in self.controller_topics.items():
            self.get_logger().info(f"    {name} -> {topic}")

    def joy_callback(self, msg: Joy) -> None:
        if self.enable_button_index < 0:
            self.enabled = False
            return

        if self.enable_button_index >= len(msg.buttons):
            self.enabled = False
            self.get_logger().warn(
                f"Joy message does not contain button index {self.enable_button_index}. "
                f"buttons length = {len(msg.buttons)}",
                throttle_duration_sec=2.0,
            )
            return

        self.enabled = bool(msg.buttons[self.enable_button_index])

    def selector_callback(self, msg: String) -> None:
        requested = msg.data.strip()

        if requested not in self.controllers:
            self.get_logger().warn(
                f"Received unknown controller '{requested}'. "
                f"Valid options: {self.controllers}",
                throttle_duration_sec=2.0,
            )
            return

        if requested != self.selected_controller:
            self.selected_controller = requested
            self.get_logger().info(
                f"Selected controller changed to '{self.selected_controller}'"
            )

    def controller_callback(self, controller_name: str, msg) -> None:
        if not self.enabled:
            return

        if controller_name != self.selected_controller:
            return

        self.drive_pub.publish(msg)

    def reset_ackermann_command(self) -> None:
        zero_command = AckermannDriveStamped()
        zero_command.drive.speed = 0
        zero_command.drive.acceleration = 0
        zero_command.drive.steering_angle = 0
        zero_command.drive.steering_angle_velocity = 0
        self.drive_pub.publish(zero_command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlGateway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_ackermann_command()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
