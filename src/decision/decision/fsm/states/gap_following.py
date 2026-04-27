from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class GapFollowingState(State):
    """Reactive state.

    If any object is detected (non-empty `objects`), remain in `StateType.REACTIVE`.
    Otherwise transition to `StateType.FP`.
    """

    _state_type = StateType(
        name="gap_following", state_traits=StateTraits.GAP_FOLLOWING
    )
    _minimum_time_in_state = 1.5

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        if objects is None or len(objects) == 0:
            return StateTraits.PURE_PURSUIT
        return self._state_type.state_traits
