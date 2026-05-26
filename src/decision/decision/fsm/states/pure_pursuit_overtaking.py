from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits
from decision.fsm.states.common import is_safe_to_overtake


class PurePursuitOvertakingState(State):
    """Overtaking state.

    This state is used when the vehicle is overtaking another vehicle with the base pure pursuit
    controller.
    """

    _state_type = StateType(
        name="pure_pursuit_overtaking",
        state_traits=StateTraits.PURE_PURSUIT | StateTraits.OVERTAKING,
    )

    _minimum_time_in_state = 1.0

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
            if is_overtake_region and opponent_distance_to_path >= 0.4:
                return StateTraits.PURE_PURSUIT | StateTraits.OVERTAKING
            elif is_overtake_region:
                return StateTraits.PURE_PURSUIT | StateTraits.TRAILING
            else: 
                return StateTraits.PURE_PURSUIT
        return self._state_type.state_traits
