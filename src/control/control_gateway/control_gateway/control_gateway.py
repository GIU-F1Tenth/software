#!/usr/bin/env python3

from functools import partial
import importlib
from typing import Dict, Optional, Type

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool


from ackermann_msgs.msg import AckermannDriveStamped


class ControlGateway(Node):
    def __init__(self) -> None:
        super().__init__("control_gateway")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("selector_topic", "/control_selector")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("enable_button_index", 4)
        self.declare_parameter(
            "manual_auto_swap_topic", "/control_gateway/switch_manual_auto"
        )
        self.declare_parameter("default_controller", "pure_pursuit")

        self.joy_topic = self.get_parameter("joy_topic").value
        self.selector_topic = self.get_parameter("selector_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.enable_button_index = int(self.get_parameter("enable_button_index").value)
        self.manual_auto_swap_topic = self.get_parameter("manual_auto_swap_topic").value
        self.default_controller = self.get_parameter("default_controller").value

        self.controller_subs = []
        self.__discover_controllers()

        self.enabled = False
        self.selected_controller: Optional[str] = None

        if self.default_controller:
            if self.default_controller not in self.controllers:
                self.get_logger().warn(
                    f"default_controller '{self.default_controller}' "
                    f"is not in controllers list: {self.controllers}"
                )
                self.__discover_controllers()
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
        self.manual_auto_swap_sub = self.create_subscription(
            Bool,
            self.manual_auto_swap_topic,
            self.manual_auto_swap_callback,
            10,
        )


        self.get_logger().info("control_gateway started")
        self.get_logger().info(f"  joy_topic: {self.joy_topic}")
        self.get_logger().info(f"  selector_topic: {self.selector_topic}")
        self.get_logger().info(f"  drive_topic: {self.drive_topic}")
        self.get_logger().info(f"  enable_button_index: {self.enable_button_index}")
        self.get_logger().info(f"  selected_controller: {self.selected_controller}")
        self.get_logger().info(f"  controllers: {self.controllers}")
        for name in self.controllers:
            self.get_logger().info(f"    {name} -> /{name}/drive")

    def __add_controller_sub(self, controller_name: str) -> None:
        sub = self.create_subscription(
                AckermannDriveStamped,
                f"/{controller_name}/drive",
                partial(self.controller_callback, controller_name),
                10,
        )
        self.controller_subs.append(sub)

    def manual_auto_swap_callback(self, msg: Bool) -> None:
        if msg.data:
            self.selected_controller = "teleop"
            self.get_logger().info("Manual/Auto swap: switched to 'teleop' controller")
            self.reset_ackermann_command()
        else:
            self.selected_controller = self.default_controller
            self.get_logger().info(
                f"Manual/Auto swap: switched to default controller '{self.default_controller}'"
            )

    def __discover_controllers(self) -> None:
        self.controllers: list[str] = []
        graph = self.get_topic_names_and_types()
        for topic_name, _ in graph:
            if topic_name.endswith("/drive"):
                controller_name = topic_name.replace("/drive", "")
                controller_name = controller_name.lstrip("/")

                if controller_name and controller_name not in self.controllers:
                    self.controllers.append(controller_name)
                    self.__add_controller_sub(controller_name)

        if not self.controllers:
            self.get_logger().warn(
                "No controllers discovered. "
                "You must provide topics in the form of '/<controller_name>/drive."
            )
            time.sleep(2.0) 
            self.__discover_controllers()

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
        if self.selected_controller == "teleop":
            return
        requested = msg.data.strip()

        if requested not in self.controllers:
            self.__discover_controllers() 
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

        if self.selected_controller == "teleop":
            self.reset_ackermann_command()

    def controller_callback(self, controller_name: str, msg) -> None:
        if not self.enabled:
            return

        if controller_name != self.selected_controller:
            return

        self.drive_pub.publish(msg)

    def reset_ackermann_command(self) -> None:
        zero_command = AckermannDriveStamped()
        zero_command.drive.speed = 0.0
        zero_command.drive.acceleration = 0.0
        zero_command.drive.steering_angle = 0.0
        zero_command.drive.steering_angle_velocity = 0.0
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
