"""
Generic Finite State Machine (FSM) Implementation

This provides a simplified, reusable state machine class that can be
used by other components, such as the MissionState manager.
"""

from typing import Dict, Set, Any, Callable, List

# Type alias for a state
State = Any
# Type alias for a listener callback
TransitionListener = Callable[[State, State], None] # (from_state, to_state)

class StateMachineError(Exception):
    """Custom exception for FSM errors."""
    pass

class StateMachine:
    """A generic, reusable Finite State Machine."""
    
    def __init__(self, initial_state: State, transitions: Dict[State, Set[State]]):
        """
        Initializes the state machine.

        Args:
            initial_state: The state to start in.
            transitions: A dictionary mapping a state to a set of
                         valid states it can transition to.
        """
        self._current_state: State = initial_state
        self._transitions: Dict[State, Set[State]] = transitions
        self._listeners: List[TransitionListener] = []

    @property
    def current(self) -> State:
        """Returns the current state."""
        return self._current_state

    def add_listener(self, listener: TransitionListener):
        """Register a callback function to be called on successful transitions."""
        self._listeners.append(listener)

    def remove_listener(self, listener: TransitionListener):
        """Unregister a transition listener."""
        try:
            self._listeners.remove(listener)
        except ValueError:
            pass # Listener not registered

    def _notify_listeners(self, from_state: State, to_state: State):
        """Notify all registered listeners of a state change."""
        for listener in self._listeners:
            try:
                listener(from_state, to_state)
            except Exception as e:
                print(f"[StateMachine] Error in listener {listener}: {e}")

    def transition(self, new_state: State) -> bool:
        """
        Attempts to transition to a new state.

        Args:
            new_state: The desired state to transition to.

        Returns:
            True if the transition was successful, False otherwise.
            
        Raises:
            StateMachineError: If a transition is attempted from a state
                               that has no defined transitions.
        """
        if self._current_state == new_state:
            return True  # No transition needed

        valid_targets = self._transitions.get(self._current_state)
        
        if valid_targets is None:
            raise StateMachineError(f"State '{self._current_state}' has no defined transitions.")

        if new_state in valid_targets:
            from_state = self._current_state
            self._current_state = new_state
            self._notify_listeners(from_state, new_state)
            return True
        else:
            print(f"[StateMachine] INVALID transition attempted: {self._current_state} -> {new_state}")
            return False
