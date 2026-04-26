from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class GapFollowingOnlyState(State):
    """Gap-Following (GF) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.GF_ONLY`.
    """

    _state_type = StateType.GF_ONLY

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        return StateType.GF_ONLY
