from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class PurePursuitOnlyState(State):
    """Follow-Path (FP) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.FP`.
    """

    _state_type = StateType.PP_ONLY

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        return StateType.PP_ONLY
