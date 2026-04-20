from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class FpState(State):
    """Follow-Path (FP) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.FP`.
    """

    _state_type = StateType.FP

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        if objects is not None and len(objects) > 0:
            return StateType.REACTIVE
        return StateType.FP

    def execute(self, objects: Optional[Collection[Any]] = None) -> None:
        pass
