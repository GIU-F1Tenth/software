#!/usr/bin/env python3

from functools import partial
from typing import Optional

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool, Float64


from ackermann_msgs.msg import AckermannDriveStamped


class ControlGateway(Node):
    def __init__(self) -> None:
        super().__init__("control_gateway")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("selector_topic", "/control_selector")
        self.declare_parameter("drive_topic", "/drive")

        self.declare_parameter(
            "manual_auto_swap_topic", "/control_gateway/switch_manual_auto"
        )
        self.declare_parameter("default_controller", "pure_pursuit")
        self.declare_parameter("trailing_topic", "/trailing")
        self.declare_parameter("speed_cap_topic", "/speed_cap")
        self.declare_parameter("controller_mode", "black")  # "gray" or "black"
        self.declare_parameter("black_controller_enable_button_index", 4)
        self.declare_parameter("gray_controller_enable_button_index", 6)

        # Keyboard control
        self.declare_parameter("is_use_keyboard_control", False)
        self.declare_parameter("keyboard_event_topic", "/keyboard/event")
        self.declare_parameter("deadman_key", "p")
        self.declare_parameter("toggle_key", "m")
        self.declare_parameter("forward_key", "w")
        self.declare_parameter("reverse_key", "s")
        self.declare_parameter("left_key", "a")
        self.declare_parameter("right_key", "d")
        self.declare_parameter("stop_key", "space")
        self.declare_parameter("keyboard_publish_rate_hz", 30.0)
        self.declare_parameter("keyboard_toggle_debounce", 0.3)
        self.declare_parameter("keyboard_max_speed", 2.0)
        self.declare_parameter("keyboard_max_reverse_speed", 1.0)
        self.declare_parameter("keyboard_max_steering", 0.34)
        self.declare_parameter("keyboard_speed_step", 0.15)
        self.declare_parameter("keyboard_steering_step", 0.04)
        self.declare_parameter("keyboard_speed_decay", 0.92)
        self.declare_parameter("keyboard_steering_return", 0.7)

        self.index = 4
        if self.get_parameter("controller_mode").value == "black":
            self.index = self.get_parameter("black_controller_enable_button_index").value
        else:
            self.index = self.get_parameter("gray_controller_enable_button_index").value

        self.joy_topic = self.get_parameter("joy_topic").value
        self.selector_topic = self.get_parameter("selector_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.enable_button_index = int(self.index)
        self.manual_auto_swap_topic = self.get_parameter("manual_auto_swap_topic").value
        self.default_controller = self.get_parameter("default_controller").value
        self.trailing_topic = self.get_parameter("trailing_topic").value
        self.speed_cap_topic = self.get_parameter("speed_cap_topic").value

        self.is_use_keyboard_control = bool(self.get_parameter("is_use_keyboard_control").value)
        self.keyboard_event_topic = self.get_parameter("keyboard_event_topic").value
        self.deadman_key = str(self.get_parameter("deadman_key").value).lower()
        self.toggle_key = str(self.get_parameter("toggle_key").value).lower()
        self.forward_key = str(self.get_parameter("forward_key").value).lower()
        self.reverse_key = str(self.get_parameter("reverse_key").value).lower()
        self.left_key = str(self.get_parameter("left_key").value).lower()
        self.right_key = str(self.get_parameter("right_key").value).lower()
        self.stop_key = str(self.get_parameter("stop_key").value).lower()
        self.keyboard_publish_rate_hz = float(self.get_parameter("keyboard_publish_rate_hz").value)
        self.keyboard_toggle_debounce = float(self.get_parameter("keyboard_toggle_debounce").value)
        self.keyboard_max_speed = float(self.get_parameter("keyboard_max_speed").value)
        self.keyboard_max_reverse_speed = float(self.get_parameter("keyboard_max_reverse_speed").value)
        self.keyboard_max_steering = float(self.get_parameter("keyboard_max_steering").value)
        self.keyboard_speed_step = float(self.get_parameter("keyboard_speed_step").value)
        self.keyboard_steering_step = float(self.get_parameter("keyboard_steering_step").value)
        self.keyboard_speed_decay = float(self.get_parameter("keyboard_speed_decay").value)
        self.keyboard_steering_return = float(self.get_parameter("keyboard_steering_return").value)

        self.controller_subs = []
        self.__discover_controllers()

        self.capped_speed = -1
        self.enabled = False
        self.should_trail = False
        self.selected_controller: Optional[str] = None

        # Keyboard state
        self._kbd_pressed_movement = set()
        self._kbd_last_toggle = 0.0
        self._kbd_speed = 0.0
        self._kbd_steering = 0.0

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
        self.hard_break_pub = self.create_publisher(
            Float64,
            "/commands/motor/brake",
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
        self.trailing_sub = self.create_subscription(
            Bool,
            self.trailing_topic,
            self.trailing_callback,
            10,
        )
        self.speed_cap_sub = self.create_subscription(
            Float64,
            self.speed_cap_topic,
            self.speed_cap_callback,
            10,
        )

        if self.is_use_keyboard_control:
            self.keyboard_event_sub = self.create_subscription(
                String,
                self.keyboard_event_topic,
                self.keyboard_event_callback,
                10,
            )
            period = 1.0 / max(self.keyboard_publish_rate_hz, 1.0)
            self.keyboard_drive_timer = self.create_timer(period, self._keyboard_drive_tick)

        self.get_logger().info("control_gateway started")
        self.get_logger().info(f"  joy_topic: {self.joy_topic}")
        self.get_logger().info(f"  selector_topic: {self.selector_topic}")
        self.get_logger().info(f"  drive_topic: {self.drive_topic}")
        self.get_logger().info(f"  enable_button_index: {self.enable_button_index}")
        self.get_logger().info(f"  selected_controller: {self.selected_controller}")
        self.get_logger().info(f"  controllers: {self.controllers}")
        if self.is_use_keyboard_control:
            self.get_logger().info(
                f"  keyboard control ON: deadman='{self.deadman_key}', toggle='{self.toggle_key}', "
                f"drive=({self.forward_key}/{self.reverse_key}/{self.left_key}/{self.right_key}), "
                f"stop='{self.stop_key}', event_topic={self.keyboard_event_topic}"
            )
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

    def speed_cap_callback(self, msg: Float64) -> None:
        self.capped_speed = msg.data

    def trailing_callback(self, msg: Bool) -> None:
        self.should_trail = msg.data

    def manual_auto_swap_callback(self, msg: Bool) -> None:
        self._set_mode_teleop(msg.data)

    def _set_mode_teleop(self, to_teleop: bool) -> None:
        if to_teleop:
            self.selected_controller = "teleop"
            self.get_logger().info("Switched to 'teleop' controller")
            self.reset_ackermann_command()
        else:
            self.selected_controller = self.default_controller
            self.get_logger().info(
                f"Switched to default controller '{self.default_controller}'"
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
        if self.is_use_keyboard_control:
            return

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

        if not self.enabled:
            self.reset_ackermann_command()

    def keyboard_event_callback(self, msg: String) -> None:
        if not self.is_use_keyboard_control:
            return

        data = msg.data.strip()
        if ":" not in data:
            return
        action, _, key = data.partition(":")
        action = action.lower()
        key = key.lower()
        if action not in ("press", "release"):
            return

        if key == self.deadman_key:
            self._handle_deadman(action == "press")
            return

        if key == self.toggle_key:
            if action == "press":
                self._handle_toggle()
            return

        if key == self.stop_key:
            if action == "press":
                self._kbd_pressed_movement.clear()
                self._kbd_speed = 0.0
                self._kbd_steering = 0.0
            return

        if key in (self.forward_key, self.reverse_key, self.left_key, self.right_key):
            if action == "press":
                self._kbd_pressed_movement.add(key)
            else:
                self._kbd_pressed_movement.discard(key)

    def _handle_deadman(self, held: bool) -> None:
        if held == self.enabled:
            return
        self.enabled = held
        if held:
            self.get_logger().info("Deadman held: gateway ENABLED")
        else:
            self.get_logger().info("Deadman released: gateway DISABLED, braking")
            self._kbd_pressed_movement.clear()
            self._kbd_speed = 0.0
            self._kbd_steering = 0.0
            self.reset_ackermann_command()

    def _handle_toggle(self) -> None:
        now = time.monotonic()
        if now - self._kbd_last_toggle < self.keyboard_toggle_debounce:
            return
        self._kbd_last_toggle = now
        to_teleop = self.selected_controller != "teleop"
        self._set_mode_teleop(to_teleop)

    def _keyboard_drive_tick(self) -> None:
        if self.forward_key in self._kbd_pressed_movement:
            self._kbd_speed += self.keyboard_speed_step
        elif self.reverse_key in self._kbd_pressed_movement:
            self._kbd_speed -= self.keyboard_speed_step
        else:
            self._kbd_speed *= self.keyboard_speed_decay
            if abs(self._kbd_speed) < 1e-3:
                self._kbd_speed = 0.0

        if self.left_key in self._kbd_pressed_movement:
            self._kbd_steering += self.keyboard_steering_step
        elif self.right_key in self._kbd_pressed_movement:
            self._kbd_steering -= self.keyboard_steering_step
        else:
            self._kbd_steering *= self.keyboard_steering_return
            if abs(self._kbd_steering) < 1e-3:
                self._kbd_steering = 0.0

        self._kbd_speed = max(
            -self.keyboard_max_reverse_speed,
            min(self.keyboard_max_speed, self._kbd_speed),
        )
        self._kbd_steering = max(
            -self.keyboard_max_steering,
            min(self.keyboard_max_steering, self._kbd_steering),
        )

        if not self.enabled:
            return
        if self.selected_controller != "teleop":
            return

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self._kbd_speed
        msg.drive.steering_angle = self._kbd_steering
        self.drive_pub.publish(msg)

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

    def controller_callback(self, controller_name: str, msg: AckermannDriveStamped) -> None:
        if not self.enabled:
            return

        if controller_name != self.selected_controller:
            return

        if self.should_trail and self.capped_speed > 0:
            msg.drive.speed = min(msg.drive.speed, self.capped_speed)

        self.drive_pub.publish(msg)

    def reset_ackermann_command(self) -> None:
        zero_command = AckermannDriveStamped()
        zero_command.drive.speed = 0.0
        zero_command.drive.acceleration = 0.0
        zero_command.drive.steering_angle = 0.0
        zero_command.drive.steering_angle_velocity = 0.0
        if self.is_use_keyboard_control: # only while using keyboard control because in sim there is no hard brake
            self.drive_pub.publish(zero_command)

        hard_break = Float64()
        hard_break.data = 100.0
        self.hard_break_pub.publish(hard_break)


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
