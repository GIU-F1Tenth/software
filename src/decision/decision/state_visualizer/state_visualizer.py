import colorsys
from typing import Dict, List, Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_PALETTE: Dict[str, Tuple[int, int, int]] = {
    "pure_pursuit_only": (0, 255, 0),
    "pure_pursuit": (0, 200, 80),
    "pure_pursuit_trailing": (255, 140, 0),
    "pure_pursuit_overtaking": (255, 0, 255),
    "gap_following": (0, 120, 255),
    "gap_following_only": (0, 60, 255),
    "gap_following_trailing": (180, 0, 255),
    "dwa": (255, 255, 0),
    "dwa_only": (200, 200, 0),
    "lqr_only": (0, 255, 255),
    "kayn_only": (255, 0, 120),
    "mpc_karim_only": (255, 60, 0),
}


def _auto_color(index: int, total: int) -> Tuple[int, int, int]:
    hue = (index / max(total, 1)) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


class StateVisualizerNode(Node):
    def __init__(self):
        super().__init__("state_visualizer_node")

        self.declare_parameter("states", ["pure_pursuit_only"])
        self.declare_parameter("state_name_topic", "state_name")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("default_color", [255, 255, 255])

        states: List[str] = list(self.get_parameter("states").value)
        state_name_topic = self.get_parameter("state_name_topic").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = int(self.get_parameter("baud_rate").value)
        serial_timeout = float(self.get_parameter("serial_timeout").value)
        default_color = tuple(int(c) for c in self.get_parameter("default_color").value)

        self._default_color: Tuple[int, int, int] = default_color
        self._color_map: Dict[str, Tuple[int, int, int]] = self.__build_color_map(states)
        self._last_color: Optional[Tuple[int, int, int]] = None

        try:
            self._serial = serial.Serial(
                serial_port, baudrate=baud_rate, timeout=serial_timeout
            )
        except serial.SerialException as exc:
            self.get_logger().error(
                f"failed to open serial port {serial_port!r}: {exc}"
            )
            self._serial = None

        self.create_subscription(
            String, state_name_topic, self.state_name_callback, 10
        )

        self.get_logger().info(
            f"state_visualizer ready: {len(self._color_map)} states on {serial_port}"
        )

    def __build_color_map(
        self, states: List[str]
    ) -> Dict[str, Tuple[int, int, int]]:
        color_map: Dict[str, Tuple[int, int, int]] = {}
        unassigned: List[str] = []
        for st in states:
            if st in DEFAULT_PALETTE:
                color_map[st] = DEFAULT_PALETTE[st]
            else:
                unassigned.append(st)

        for i, st in enumerate(unassigned):
            color_map[st] = _auto_color(i, len(unassigned))
        return color_map

    def state_name_callback(self, msg: String) -> None:
        state_name = msg.data
        color = self._color_map.get(state_name, self._default_color)

        if color == self._last_color:
            return
        self._last_color = color
        self.__send_color(color, state_name)

    def __send_color(self, color: Tuple[int, int, int], state_name: str) -> None:
        if self._serial is None:
            return

        r, g, b = color
        payload = f"<{r},{g},{b}>\n".encode("ascii")
        try:
            self._serial.write(payload)
            self._serial.flush()
        except serial.SerialException as exc:
            self.get_logger().error(f"serial write failed: {exc}")
            return

        self.get_logger().info(
            f"state={state_name} color=({r},{g},{b})", throttle_duration_sec=1.0
        )

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
