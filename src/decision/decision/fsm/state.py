from abc import ABC, abstractmethod
from enum import Enum
from typing import Final, Collection, Any



class StateType(Enum):
    """Enumeration of possible FSM state types."""
    IDLE = "idle"
    FP = "fp"
    REACTIVE = "reactive"

class State(ABC):
    """
    Abstract base class for FSM states.
    
    All state implementations must inherit from this class and implement
    the required abstract methods.
    """
    _state_type: Final[StateType]

    @property
    def state_type(self) -> StateType:
        """Get the state type."""
        return self._state_type

    @abstractmethod
    def transition(self, objects: Collection[Any] = None) -> StateType:
        """
        Determine the next state to transition to.

        This method should contain the logic to decide whether to transition
        to another state and which state to transition to.

        Returns:
            The next State to transition to as a StateType.
        """
        pass
    
    @abstractmethod
    def execute(self, objects: Collection[Any] = None) -> None:
        """
        Execute the actions associated with this state.

        This method should contain the logic to perform the actions that are
        specific to this state.
        """
        pass
