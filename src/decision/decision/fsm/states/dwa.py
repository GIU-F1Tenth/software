from typing import Collection, Any, Optional

from decision.fsm.state import State, StateType, StateTraits


class DwaState(State):
    """Reactive state.

    If any object is detected (non-empty `objects`), remain in `StateType.REACTIVE`.
    Otherwise transition to `StateType.FP`.
    """

    _state_type = StateType(name="dwa", state_traits=StateTraits.DWA)
    _minimum_time_in_state = 2.4

    @property
    def state_type(self) -> StateType:
        return self._state_type

    def transition(
        self,
        objects: Optional[Collection[Any]] = None,
        is_overtake_region: bool = False,
        opponent_distance_to_path: float = float("inf"),
    ) -> StateTraits:
        if objects is None or len(objects) == 0 or not is_overtake_region:
            return StateTraits.PURE_PURSUIT | StateTraits.TRAILING
        return self._state_type.state_traits
