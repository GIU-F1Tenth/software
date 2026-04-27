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

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateTraits:
        safe_to_overtake = True  # TODO: implement safety assessment logic

        if objects is not None and len(objects) > 0:
            if safe_to_overtake:
                return StateTraits.GAP_FOLLOWING
            else:
                return self._state_type.state_traits
        else:
            return StateTraits.PURE_PURSUIT
