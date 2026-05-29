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
    "pure_pursuit_overtaking": (255, 0, 0),
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

        self.declare_parameter("states", ["pure_pursuit_only", "pure_pursuit", "pure_pursuit_trailing", "pure_pursuit_overtaking",])
        self.declare_parameter("state_name_topic", "state_name")
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("default_color", [255, 255, 255])
        self.declare_parameter("flashing_states", ["pure_pursuit_overtaking"])
        self.declare_parameter("flash_hz", 6.0)
        self.declare_parameter("beeping_states", ["pure_pursuit_overtaking"])
        self.declare_parameter("beep_freq", 2000)
        self.declare_parameter("beep_ms", 90)

        states: List[str] = list(self.get_parameter("states").value)
        state_name_topic = self.get_parameter("state_name_topic").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = int(self.get_parameter("baud_rate").value)
        serial_timeout = float(self.get_parameter("serial_timeout").value)
        default_color = tuple(int(c) for c in self.get_parameter("default_color").value)

        self._default_color: Tuple[int, int, int] = default_color
        self._color_map: Dict[str, Tuple[int, int, int]] = self.__build_color_map(states)

        self._flashing_states: set = set(self.get_parameter("flashing_states").value)
        flash_hz = float(self.get_parameter("flash_hz").value)

        self._beeping_states: set = set(self.get_parameter("beeping_states").value)
        self._beep_freq: int = int(self.get_parameter("beep_freq").value)
        self._beep_ms: int = int(self.get_parameter("beep_ms").value)

        self._active_color: Optional[Tuple[int, int, int]] = None
        self._active_state: Optional[str] = None
        self._is_flashing: bool = False
        self._is_beeping: bool = False
        self._phase_on: bool = True

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

        flash_hz = max(flash_hz, 0.1)
        self._effect_timer = self.create_timer(0.5 / flash_hz, self.__effect_tick)

        self.get_logger().info(
            f"state_visualizer ready: {len(self._color_map)} states on {serial_port}, "
            f"flashing={sorted(self._flashing_states)} at {flash_hz} Hz, "
            f"beeping={sorted(self._beeping_states)} at {self._beep_freq} Hz/{self._beep_ms} ms"
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
        self.get_logger().info(
            f"assigned colors: {color_map}, default_color={self._default_color}"
        )
        return color_map

    def state_name_callback(self, msg: String) -> None:
        state_name = msg.data
        color = self._color_map.get(state_name, self._default_color)

        if state_name == self._active_state and color == self._active_color:
            return

        self._active_state = state_name
        self._active_color = color
        self._is_flashing = state_name in self._flashing_states
        self._is_beeping = state_name in self._beeping_states
        self._phase_on = True
        self.__send_color(color, state_name)

    def __effect_tick(self) -> None:
        if self._active_color is None:
            return

        self._phase_on = not self._phase_on

        if self._is_flashing:
            color = self._active_color if self._phase_on else (0, 0, 0)
            self.__send_color(color, self._active_state or "")

        if self._is_beeping and self._phase_on:
            self.__send_beep()

    def __send_beep(self) -> None:
        if self._serial is None:
            return

        payload = f"[{self._beep_freq},{self._beep_ms}]\n".encode("ascii")
        try:
            self._serial.write(payload)
            self._serial.flush()
        except serial.SerialException as exc:
            self.get_logger().error(f"serial beep write failed: {exc}")

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
