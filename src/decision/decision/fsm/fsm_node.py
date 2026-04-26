from collections.abc import Collection
from importlib import import_module
from typing import Dict, Optional

from decision.fsm.state import State, StateType
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from decision.fsm import FSM


class SimpleFSM(FSM):
    """Finite State Machine.

    States are expected to live in modules named `decision.fsm.states.<value>`
    where `<value>` is the `StateType.value` (for example "idle"). The class
    name inside the module is expected to be the PascalCase form of the value
    with a trailing `State` (for example `IdleState`).
    """

    def __init__(self, state_types: Collection[StateType], initial: StateType):
        """Load state instances and set the initial current state.

        Args:
            state_types: collection of StateType values to load into the FSM.
            initial: the initial StateType to set as the current state.
        """
        self._state_by_type: Dict[StateType, State] = {}

        if initial not in state_types:
            raise ValueError("initial state must be one of the provided state_types")

        for st in state_types:
            module_name = st.value
            class_name = (
                "".join(part.capitalize() for part in module_name.split("_")) + "State"
            )
            try:
                module = import_module(f"decision.fsm.states.{module_name}")
                cls = getattr(module, class_name)
                instance = cls()
            except Exception as exc:
                raise ImportError(f"failed to load state {st!r}: {exc}") from exc

            if not isinstance(instance, State):
                raise TypeError(f"state {st!r} instance is not a subclass of State")
            self._state_by_type[st] = instance

        self._current_state: State = self._state_by_type[initial]

    @property
    def current_state(self) -> State:
        """Return the active state instance."""
        return self._current_state

    def run_once(self, objects: Optional[Collection]) -> None:
        """Perform one execution and transition step.

        - Calls the current state's `execute`.
        - Calls the current state's `transition` to obtain the next
          StateType.
        - Updates the current state by reusing the already-created instance
          from the internal pool.

        Raises:
            ValueError: if the returned StateType is not part of the pool.
        """
        next_type = self._current_state.transition(objects=objects)

        if not isinstance(next_type, StateType):
            raise ValueError("transition must return a StateType")
        if next_type not in self._state_by_type:
            raise ValueError(f"state {next_type!r} is not present in FSM pool")

        self._current_state = self._state_by_type[next_type]


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        self.declare_parameter("states", ["idle"])
        self.declare_parameter("initial_state", "idle")
        self.declare_parameter("objects_topic", "objects")
        self.declare_parameter("output_topic", "fsm_state")

        states_list = self.get_parameter("states").value
        initial_state = self.get_parameter("initial_state").value
        objects_topic = self.get_parameter("objects_topic").value
        output_topic = self.get_parameter("output_topic").value

        state_types = [StateType(s) for s in states_list]
        initial = StateType(initial_state)

        self.fsm = SimpleFSM(state_types, initial)

        self.publisher = self.create_publisher(String, output_topic, 10)

        self.subscription = self.create_subscription(
            MarkerArray, objects_topic, self.objects_callback, 10
        )

    def objects_callback(self, msg):
        self.fsm.run_once(objects=msg.markers)
        state_str = self.fsm.current_state.state_type.value
        self.get_logger().info(
            f"Current FSM state: {state_str}", throttle_duration_sec=1.0
        )
        output_msg = String()
        output_msg.data = state_str
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
