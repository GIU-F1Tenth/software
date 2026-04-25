from abc import ABC, abstractmethod
from typing import Collection, Optional

from decision.fsm.state import State


class FSM(ABC):
    @property
    @abstractmethod
    def current_state(self) -> State:
        """Return the active state instance."""

    @abstractmethod
    def run_once(self, objects: Optional[Collection]) -> None:
        """Perform one execution and transition step."""
