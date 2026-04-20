from decision.fsm.state import State, StateType

class IdleState(State):
    """
    Idle state implementation.
    
    This state represents an idle/waiting state where the system is not
    performing any active operations.
    """

    _state_type = StateType.IDLE

    @property
    def state_type(self) -> StateType:
        """Get the state type."""
        return self._state_type

    def transition(self) -> StateType:
        """
        Determine the next state to transition to.

        In the idle state, there is no automatic transition.
        Transitions would typically be triggered by external events.

        Returns:
            StateType.IDLE to remain in the idle state.
        """
        return StateType.ACTIVE

    def execute(self) -> None:
        """
        Execute the actions associated with the idle state.

        In the idle state, no active operations are performed.
        This is a waiting/standby state.
        """
        pass
