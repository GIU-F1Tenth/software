from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class ReactiveState(State):
    """Reactive state.

    If any object is detected (non-empty `objects`), remain in `StateType.REACTIVE`.
    Otherwise transition to `StateType.FP`.
    """

    _state_type = StateType.REACTIVE

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        if objects is None or len(objects) == 0:
            return StateType.FP
        return StateType.REACTIVE
