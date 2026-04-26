#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from time import time


class TeleopSwitcher(Node):
    def __init__(self) -> None:
        super().__init__("teleop_switcher")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("selector_topic", "/control_selector")
        self.declare_parameter("debounce_time", 0.5)

        self.declare_parameter("manual_button_index", 7)

        self.joy_topic = self.get_parameter("joy_topic").value
        self.selector_topic = self.get_parameter("selector_topic").value
        self.manual_button_index = int(self.get_parameter("manual_button_index").value)
        self.debounce_time = float(self.get_parameter("debounce_time").value)

        self.selector_pub = self.create_publisher(Bool, self.selector_topic, 10)
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

        self.last_pressed = 0
        self.manual_mode = True

    def joy_callback(self, msg: Joy) -> None:
        current_time = time()

        if current_time - self.last_pressed < self.debounce_time:
            return

        self.last_pressed = current_time

        if self.manual_button_index < 0:
            return

        if self.manual_button_index >= len(msg.buttons):
            self.prev_buttons = list(msg.buttons)
            return

        if bool(msg.buttons[self.manual_button_index]):
            self.manual_mode = not self.manual_mode
            mode_str = "teleop" if self.manual_mode else "autonomous"
            self.get_logger().info(f"Switching to {mode_str} mode")
            self.selector_pub.publish(Bool(data=self.manual_mode))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopSwitcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
