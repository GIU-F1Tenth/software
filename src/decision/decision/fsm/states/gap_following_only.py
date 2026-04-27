from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class GapFollowingOnlyState(State):
    """Gap-Following (GF) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.GF_ONLY`.
    """

    _state_type = StateType(
        name="gap_following_only", state_traits=StateTraits.GAP_FOLLOWING
    )

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        return self._state_type.state_traits
