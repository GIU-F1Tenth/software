#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pynput import keyboard


_MODIFIER_ALIASES = {
    "shift_l": "shift",
    "shift_r": "shift",
    "ctrl_l": "ctrl",
    "ctrl_r": "ctrl",
    "alt_l": "alt",
    "alt_r": "alt",
    "cmd": "cmd",
    "cmd_l": "cmd",
    "cmd_r": "cmd",
}


def key_to_name(key) -> str:
    char = getattr(key, "char", None)
    if char is not None:
        return char.lower()
    name = getattr(key, "name", None)
    if name is not None:
        name = name.lower()
        return _MODIFIER_ALIASES.get(name, name)
    return ""


class KeyboardSwitcher(Node):
    """Publishes raw keyboard press/release events as std_msgs/String.

    Message format: 'press:<key>' or 'release:<key>'.
    All interpretation (deadman, toggle, drive math) lives in control_gateway.
    """

    def __init__(self) -> None:
        super().__init__("keyboard_switcher")

        self.declare_parameter("event_topic", "/keyboard/event")
        self.event_topic = self.get_parameter("event_topic").value

        self.event_pub = self.create_publisher(String, self.event_topic, 10)

        self._last_pressed = set()

        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.daemon = True
        self.listener.start()

        self.get_logger().info(
            f"keyboard_switcher started; publishing key events on {self.event_topic}"
        )

    def _on_press(self, key) -> None:
        name = key_to_name(key)
        if not name:
            return
        if name in self._last_pressed:
            return
        self._last_pressed.add(name)
        self.event_pub.publish(String(data=f"press:{name}"))

    def _on_release(self, key) -> None:
        name = key_to_name(key)
        if not name:
            return
        self._last_pressed.discard(name)
        self.event_pub.publish(String(data=f"release:{name}"))

    def destroy_node(self) -> bool:
        try:
            self.listener.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
