from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class KaynOnlyState(State):
    """Follow-Path (FP) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.FP`.
    """

    _state_type = StateType(
        name="kayn_only", state_traits=StateTraits.KAYN
    )

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        return self._state_type.state_traits
