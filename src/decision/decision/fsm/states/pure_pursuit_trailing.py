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

    _gap_following_state = StateType(
        name="gap_following", state_traits=StateTraits.GAP_FOLLOWING
    )
    _pure_pursuit = StateType(
        name="pure_pursuit", state_traits=StateTraits.PURE_PURSUIT
    )

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        safe_to_overtake = True  # TODO: implement safety assessment logic

        if objects is not None and len(objects) > 0:
            if safe_to_overtake:
                return self._gap_following_state
            else:
                return self._state_type
        else:
            return self._pure_pursuit
