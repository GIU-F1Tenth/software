from typing import Collection, Any, Optional

from decision.fsm.states.common import is_safe_to_overtake
from decision.fsm.state import State, StateType, StateTraits


class PurePursuitState(State):
    """Follow-Path (FP) state.

    If any object is detected (non-empty `objects`), transition to
    `StateType.REACTIVE`. Otherwise remain in `StateType.FP`.
    """

    _state_type = StateType(name="pure_pursuit", state_traits=StateTraits.PURE_PURSUIT)

    _gap_following = StateType(
        name="gap_following", state_traits=StateTraits.GAP_FOLLOWING
    )

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(
        self,
        objects: Optional[Collection[Any]] = None,
        is_overtake_region: bool = False,
        opponent_distance_to_path: float = float("inf"),
    ) -> StateTraits:
        if objects is not None and len(objects) > 0:
            if is_overtake_region and is_safe_to_overtake():
                return self._gap_following.state_traits
            if opponent_distance_to_path < 0.4:
                return self._state_type.state_traits | StateTraits.TRAILING
        return self._state_type.state_traits
