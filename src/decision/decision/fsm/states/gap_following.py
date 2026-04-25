from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class GapFollowingState(State):
    """Reactive state.

    If any object is detected (non-empty `objects`), remain in `StateType.REACTIVE`.
    Otherwise transition to `StateType.FP`.
    """

    _state_type = StateType.GF

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        if objects is None or len(objects) == 0:
            return StateType.PP
        return StateType.GF
