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

    _pp_state = StateType(name="pure_pursuit", state_traits=StateTraits.PURE_PURSUIT)

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        if objects is None or len(objects) == 0:
            return self._pp_state
        return self._state_type
