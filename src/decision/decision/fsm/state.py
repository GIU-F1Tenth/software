from abc import ABC, abstractmethod
from enum import IntFlag, auto
from typing import Collection

from dataclasses import dataclass


class StateTraits(IntFlag):
    PURE_PURSUIT = auto()
    GAP_FOLLOWING = auto()
    DWA = auto()
    LQR = auto()
    KAYN = auto()
    
    TRAILING = auto()
    STOP = auto()


@dataclass(frozen=True)
class StateType:
    """Enumeration of possible FSM state types."""

    name: str
    state_traits: StateTraits

    def __eq__(self, other: object):
        if not isinstance(other, StateType):
            return NotImplemented

        return self.state_traits == other.state_traits


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
    def transition(self, objects: Collection | None = None) -> StateTraits:
        """
        Determine the next state to transition to.

        This method should contain the logic to decide whether to transition
        to another state and which state to transition to.

        Returns:
            The next State to transition to as a StateType.
        """
        pass
