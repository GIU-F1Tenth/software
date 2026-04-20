from importlib import import_module
from typing import Collection, Dict, Any

from decision.fsm.state import StateType, State


class FSM:
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
            class_name = "".join(part.capitalize() for part in module_name.split("_")) + "State"
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

    def run_once(self) -> None:
        """Perform one execution and transition step.

        - Calls the current state's `execute`.
        - Calls the current state's `transition` to obtain the next
          StateType.
        - Updates the current state by reusing the already-created instance
          from the internal pool.

        Raises:
            ValueError: if the returned StateType is not part of the pool.
        """
        self._current_state.execute()
        next_type = self._current_state.transition()
        
        if not isinstance(next_type, StateType):
            raise ValueError("transition must return a StateType")
        if next_type not in self._state_by_type:
            raise ValueError(f"state {next_type!r} is not present in FSM pool")

        self._current_state = self._state_by_type[next_type]
