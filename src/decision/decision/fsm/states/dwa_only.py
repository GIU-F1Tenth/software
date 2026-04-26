from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class DwaOnlyState(State):
    """DWA (Differential Drive) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.DWA_ONLY`.
    """

    _state_type = StateType.DWA_ONLY

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        return StateType.DWA_ONLY
