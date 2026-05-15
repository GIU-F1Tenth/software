from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class PurePursuitTrailingState(State):
    """Trailing state.

    This state is used when the vehicle is following another vehicle with the base pure pursuit
    controller.
    """

    _state_type = StateType(
        name="pure_pursuit_trailing",
        state_traits=StateTraits.PURE_PURSUIT | StateTraits.TRAILING,
    )
    
    _minimum_time_in_state = 5.0

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None, is_overtake_region: bool = False) -> StateTraits:
        if objects is not None and len(objects) > 0:
            if is_overtake_region and self.__is_safe_to_overtake():
                return StateTraits.GAP_FOLLOWING    
        return self._state_type.state_traits
    
    def __is_safe_to_overtake(self):
        return False