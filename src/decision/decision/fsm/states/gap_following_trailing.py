from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class GapFollowingTrailingState(State):
    """Gap-Following (GF) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.GF_ONLY`.
    """

    _state_type = StateType(
        name="gap_following_trailing", state_traits=StateTraits.GAP_FOLLOWING | StateTraits.TRAILING
    )

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        return self._state_type.state_traits
