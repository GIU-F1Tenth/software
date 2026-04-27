from typing import Collection, Any, Optional

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

    def transition(self, objects: Optional[Collection[Any]] = None) -> StateType:
        if objects is not None and len(objects) > 0:
            return self._gap_following
        return self._state_type
