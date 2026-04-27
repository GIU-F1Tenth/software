from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class DwaOnlyState(State):
    """DWA (Differential Drive) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.DWA_ONLY`.
    """

    _state_type = StateType(name="dwa_only", state_traits=StateTraits.DWA)

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        return self._state_type.state_traits
