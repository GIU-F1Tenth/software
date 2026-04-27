from collections.abc import Collection
from importlib import import_module
from typing import Dict, Optional

from decision.fsm.state import State, StateType, StateTraits
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray

import time
from decision.fsm import FSM


class SimpleFSM(FSM):
    """Finite State Machine.

    States are expected to live in modules named `decision.fsm.states.<value>`
    where `<value>` is the `StateType.value` (for example "idle"). The class
    name inside the module is expected to be the PascalCase form of the value
    with a trailing `State` (for example `IdleState`).
    """

    def __init__(self, state_types: Collection[str], initial: str):
        """Load state instances and set the initial current state.

        Args:
            state_types: collection of StateType values to load into the FSM.
            initial: the initial StateType to set as the current state.
        """
        self._state_by_state_traits: Dict[StateTraits, State] = {}
        self.__state_time = time.perf_counter()

        if initial not in state_types:
            raise ValueError("initial state must be one of the provided state_types")

        for st in state_types:
            module_name = st
            class_name = (
                "".join(part.capitalize() for part in module_name.split("_")) + "State"
            )
            try:
                module = import_module(f"decision.fsm.states.{module_name}")
                cls = getattr(module, class_name)
                instance = cls()
                
                if st == initial:
                    initial_state = instance
            except Exception as exc:
                raise ImportError(f"failed to load state {st!r}: {exc}") from exc

            if not isinstance(instance, State):
                raise TypeError(f"state {st!r} instance is not a subclass of State")
            self._state_by_state_traits[instance.state_type.state_traits] = instance

        self._current_state: State = self._state_by_state_traits[initial_state.state_type.state_traits]

    @property
    def current_state(self) -> State:
        """Return the active state instance."""
        return self._current_state

    def run_once(self, objects: Optional[Collection]):
        """Perform one execution and transition step.

        - Calls the current state's `execute`.
        - Calls the current state's `transition` to obtain the next
          StateType.
        - Updates the current state by reusing the already-created instance
          from the internal pool.

        Raises:
            ValueError: if the returned StateType is not part of the pool.
        """
        elapsed_time = time.perf_counter() - self.__state_time
        next_type_traits = self._current_state.transition(objects=objects)

        if not isinstance(next_type_traits, StateTraits):
            raise ValueError("transition must return a StateType")
        if next_type_traits not in self._state_by_state_traits:
            raise ValueError(f"state {next_type_traits!r} is not present in FSM pool")

        if self.__should_switch_state(next_type_traits, elapsed_time):
            self.__state_time = time.perf_counter()
            self._current_state = self._state_by_state_traits[next_type_traits]

        return elapsed_time

    def __should_switch_state(
        self, next_type: StateTraits, elapsed_time: float
    ) -> bool:
        if (
            next_type != self._current_state.state_type.state_traits
            and elapsed_time > self._current_state.minimum_time_in_state
        ):
            return True
        return False


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        self.declare_parameter("states", ["idle"])
        self.declare_parameter("initial_state", "idle")
        self.declare_parameter("objects_topic", "objects")
        self.declare_parameter("output_topic", "fsm_state")
        self.declare_parameter("trailing_topic", "trailing")

        states_list = self.get_parameter("states").value
        initial_state = self.get_parameter("initial_state").value
        objects_topic = self.get_parameter("objects_topic").value
        control_output_topic = self.get_parameter("output_topic").value
        trailing_topic = self.get_parameter("trailing_topic").value

        self.fsm = SimpleFSM(states_list, initial_state)

        self.control_publisher = self.create_publisher(String, control_output_topic, 10)
        self.trailing_topic = self.create_publisher(Bool, trailing_topic, 10)

        self.subscription = self.create_subscription(
            MarkerArray, objects_topic, self.objects_callback, 10
        )

    def objects_callback(self, msg):
        self.fsm.run_once(objects=msg.markers[1:])
        state_str = self.fsm.current_state.state_type.name
        self.get_logger().info(
            f"Current FSM state: {state_str}", throttle_duration_sec=1.0
        )
        control_output_msg = String()
        control_output_msg.data = state_str
        self.control_publisher.publish(control_output_msg)
        
        trail_output_msg = Bool()
        trail_output_msg.data = StateTraits.TRAILING in self.fsm.current_state.state_type.state_traits
        self.trailing_topic.publish(trail_output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
