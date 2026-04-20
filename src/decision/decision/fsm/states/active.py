from decision.fsm.state import State, StateType

class ActiveState(State):
    """
    Active state implementation.
    
    This state represents an active state where the system is performing
    active operations.
    """

    _state_type = StateType.ACTIVE

    @property
    def state_type(self) -> StateType:
        """Get the state type."""
        return self._state_type

    def transition(self) -> StateType:
        """
        Determine the next state to transition to.

        In the active state, there is no automatic transition.
        Transitions would typically be triggered by external events.

        Returns:
            StateType.ACTIVE to remain in the active state.
        """
        return StateType.ACTIVE

    def execute(self) -> None:
        """
        Execute the actions associated with the active state.

        In the active state, the system performs its main operations.
        """
        pass
