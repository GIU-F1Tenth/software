#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class TeleopSwitcher(Node):
    def __init__(self) -> None:
        super().__init__("teleop_switcher")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("selector_topic", "/control_selector")
        self.declare_parameter("manual_controller_name", "teleop")

        self.declare_parameter("manual_button_index", 1)

        self.joy_topic = self.get_parameter("joy_topic").value
        self.selector_topic = self.get_parameter("selector_topic").value
        self.manual_controller_name = self.get_parameter(
            "manual_controller_name"
        ).value
        self.manual_button_index = int(
            self.get_parameter("manual_button_index").value
        )

        self.selector_pub = self.create_publisher(String, self.selector_topic, 10)
        self.joy_sub = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

    def joy_callback(self, msg: Joy) -> None:
        if self.manual_button_index < 0:
            return

        if self.manual_button_index >= len(msg.buttons):
            self.get_logger().warn(
                f"Joy message does not contain button index {self.manual_button_index}. "
                f"buttons length = {len(msg.buttons)}",
                throttle_duration_sec=2.0,
            )
            self.prev_buttons = list(msg.buttons)
            return

        if bool(msg.buttons[self.manual_button_index]):
            out = String()
            out.data = self.manual_controller_name
            self.selector_pub.publish(out)
            self.get_logger().info(
                f"Manual override requested, switching selector to "
                f"'{self.manual_controller_name}'"
            )


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