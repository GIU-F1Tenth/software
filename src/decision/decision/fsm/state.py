from abc import ABC, abstractmethod
from enum import Enum
from typing import Collection


class StateType(Enum):
    """Enumeration of possible FSM state types."""

    # Normal states
    PP = "pure_pursuit"
    GF = "gap_following"
    PP_TRAILING = "pp_trailing"

    # Single state modes for testing
    PP_ONLY = "pure_pursuit_only"
    GF_ONLY = "gap_following_only"
    DWA_ONLY = "dwa_only"
    LQR_ONLY = "lqr_only"


class State(ABC):
    """
    Abstract base class for FSM states.

    All state implementations must inherit from this class and implement
    the required abstract methods.
    """

    _state_type: StateType
    _minimum_time_in_state: float = 0.0

    @property
    def state_type(self) -> StateType:
        """Get the state type."""
        return self._state_type

    @property
    def minimum_time_in_state(self) -> float:
        """Get the minimum time (in seconds) that must be spent in this state before transitioning."""
        return self._minimum_time_in_state

    @abstractmethod
    def transition(self, objects: Collection | None = None) -> StateType:
        """
        Determine the next state to transition to.

        This method should contain the logic to decide whether to transition
        to another state and which state to transition to.

        Returns:
            The next State to transition to as a StateType.
        """
        pass
