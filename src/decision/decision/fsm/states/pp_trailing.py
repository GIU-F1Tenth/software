from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType


class PpTrailingState(State):
    """Trailing state.

    This state is used when the vehicle is following another vehicle with the base pure pursuit
    controller.
    """

    _state_type = StateType.PP_TRAILING

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        safe_to_overtake = True  # TODO: implement safety assessment logic

        if objects is not None and len(objects) > 0:
            if safe_to_overtake:
                return StateType.GF
            else:
                return StateType.PP_TRAILING
        else:
            return StateType.PP
