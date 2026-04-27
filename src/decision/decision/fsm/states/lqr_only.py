from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class LqrOnlyState(State):
    """LQR (Linear Quadratic Regulator) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.LQR_ONLY`.
    """

    _state_type = StateType(name="lqr_only", state_traits=StateTraits.LQR)

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        return self._state_type
