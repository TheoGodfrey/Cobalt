"""
Component 5: Mission State
Logical mission progress tracker and state machine.

This component is responsible for tracking the logical state of the mission,
validating transitions, and logging the history of states, as specified
in the COBALT architecture.

This version is refactored to use the generic StateMachine class.
"""

import time
from enum import Enum, auto
from typing import List, Tuple, Optional, Callable, Set, Dict

from .state_machine import StateMachine, StateMachineError

class MissionStateEnum(Enum):
    """Enumeration of all possible logical mission states."""
    IDLE = auto()
    STARTING = auto()
    SEARCHING = auto()
    CONFIRMING = auto() # e.g., moving closer to a detection
    DELIVERING = auto()
    RETURNING = auto()  # Standard Return-to-Hub (RTH)
    PAUSED = auto()
    EMERGENCY_RTH = auto() # Failsafe RTH
    LANDING = auto()
    ABORTED = auto()
    MISSION_COMPLETE = auto()
    
    # Add PATROLLING as requested by mission_controller
    PATROLLING = auto() 

# --- FIX for Bug #11: Add explicit mappings ---

# A mapping from mission file phase names (strings) to enum members
MISSION_PHASE_TO_ENUM: Dict[str, MissionStateEnum] = {
    "idle": MissionStateEnum.IDLE,
    "starting": MissionStateEnum.STARTING,
    "searching": MissionStateEnum.SEARCHING,
    "confirming": MissionStateEnum.CONFIRMING,
    "delivering": MissionStateEnum.DELIVERING,
    "returning": MissionStateEnum.RETURNING,
    "paused": MissionStateEnum.PAUSED,
    "emergency_rth": MissionStateEnum.EMERGENCY_RTH,
    "landing": MissionStateEnum.LANDING,
    "aborted": MissionStateEnum.ABORTED,
    "mission_complete": MissionStateEnum.MISSION_COMPLETE,
    "patrolling": MissionStateEnum.PATROLLING,
}

# A reverse mapping from enum members to phase names (strings)
MISSION_ENUM_TO_PHASE: Dict[MissionStateEnum, str] = {
    v: k for k, v in MISSION_PHASE_TO_ENUM.items()
}

# --- End of FIX for Bug #11 ---


# Type alias for a state log entry: (timestamp, from_state, to_state)
StateLogEntry = Tuple[float, MissionStateEnum, MissionStateEnum]

# Type alias for a state change listener: Callable[[MissionStateEnum], None]
# Note: The generic listener is (from, to), we adapt it.
StateChangeListener = Callable[[MissionStateEnum], None]


def get_mission_transitions() -> Dict[MissionStateEnum, Set[MissionStateEnum]]:
    """Builds and returns the transition map for the mission FSM."""
    
    transitions: Dict[MissionStateEnum, Set[MissionStateEnum]] = {
        MissionStateEnum.IDLE: {
            MissionStateEnum.STARTING,
            MissionStateEnum.SEARCHING,  # <-- ADD THIS
            MissionStateEnum.PATROLLING, # <-- ADD THIS
            MissionStateEnum.DELIVERING
        },
        MissionStateEnum.STARTING: {
            MissionStateEnum.SEARCHING, 
            MissionStateEnum.PATROLLING, 
            MissionStateEnum.DELIVERING, 
            MissionStateEnum.ABORTED
        },
        MissionStateEnum.SEARCHING: {
            MissionStateEnum.CONFIRMING, 
            MissionStateEnum.RETURNING, 
            MissionStateEnum.PATROLLING
        },
        MissionStateEnum.CONFIRMING: {
            MissionStateEnum.SEARCHING, 
            MissionStateEnum.DELIVERING
        },
        MissionStateEnum.DELIVERING: {
            MissionStateEnum.SEARCHING, 
            MissionStateEnum.RETURNING,
            MissionStateEnum.PATROLLING
        },
        MissionStateEnum.PATROLLING: {
            MissionStateEnum.SEARCHING,
            MissionStateEnum.RETURNING
        },
        MissionStateEnum.RETURNING: {
            MissionStateEnum.LANDING
        },
        MissionStateEnum.LANDING: {
            MissionStateEnum.MISSION_COMPLETE, 
            MissionStateEnum.IDLE, 
        },
        MissionStateEnum.PAUSED: {
            MissionStateEnum.SEARCHING, 
            MissionStateEnum.CONFIRMING, 
            MissionStateEnum.DELIVERING, 
            MissionStateEnum.RETURNING, 
            MissionStateEnum.PATROLLING
        },
        MissionStateEnum.EMERGENCY_RTH: {
            MissionStateEnum.LANDING, 
            MissionStateEnum.ABORTED
        },
        MissionStateEnum.ABORTED: {
            MissionStateEnum.IDLE
        },
        MissionStateEnum.MISSION_COMPLETE: {
            MissionStateEnum.IDLE
        }
    }
    
    # Add global transitions (PAUSE, ABORT, EMERGENCY_RTH) to all active states
    global_transitions = {
        MissionStateEnum.PAUSED, 
        MissionStateEnum.EMERGENCY_RTH, 
        MissionStateEnum.ABORTED
    }
    active_states = {
        MissionStateEnum.STARTING,
        MissionStateEnum.SEARCHING, 
        MissionStateEnum.CONFIRMING,
        MissionStateEnum.DELIVERING, 
        MissionStateEnum.RETURNING,
        MissionStateEnum.LANDING,
        MissionStateEnum.PATROLLING
    }
    
    for state in active_states:
        if state in transitions:
            transitions[state].update(global_transitions)
        else:
            transitions[state] = global_transitions
            
    return transitions


class MissionState:
    """
    Manages the logical mission state, validates transitions,
    and logs state history. Wraps the generic StateMachine.
    """
    
    def __init__(self, start_state: MissionStateEnum = MissionStateEnum.IDLE):
        self._state_history: List[StateLogEntry] = []
        self._listeners: Set[StateChangeListener] = set()
        self._previous_state: Optional[MissionStateEnum] = None
        
        # Instantiate the generic FSM
        self._fsm = StateMachine(
            initial_state=start_state,
            transitions=get_mission_transitions()
        )
        
        # Add our internal logger as a listener
        self._fsm.add_listener(self._log_and_notify)
        
        # Log the initial state
        self._log_state(None, start_state)

    def _log_state(self, from_state: Optional[MissionStateEnum], to_state: MissionStateEnum):
        """Internal helper to log state changes."""
        timestamp = time.monotonic()
        # The FSM only logs transitions, so we log the initial state manually
        if from_state is not None:
            self._state_history.append((timestamp, from_state, to_state))
            print(f"[MissionState] Transition: {from_state.name} -> {to_state.name}")
        else:
            print(f"[MissionState] Initial state: {to_state.name}")
            
    def _log_and_notify(self, from_state: MissionStateEnum, to_state: MissionStateEnum):
        """
        Internal listener passed to the generic FSM.
        It logs the state and notifies external listeners.
        """
        self._log_state(from_state, to_state)
        
        # Store previous state for pause/resume logic
        if to_state != MissionStateEnum.PAUSED:
            self._previous_state = from_state
        
        # Notify external listeners
        for listener in self._listeners:
            try:
                listener(to_state) # Pass only the new state
            except Exception as e:
                print(f"[MissionState] Error in listener {listener}: {e}")

    def add_listener(self, listener: StateChangeListener):
        """Register a callback function to be called on state changes."""
        self._listeners.add(listener)

    def remove_listener(self, listener: StateChangeListener):
        """Unregister a state change listener."""
        self._listeners.discard(listener)

    def transition(self, new_state: MissionStateEnum) -> bool:
        """
        Attempts to transition to a new state.

        Args:
            new_state: The desired MissionStateEnum to transition to.

        Returns:
            True if the transition was successful, False otherwise.
        """
        try:
            return self._fsm.transition(new_state)
        except StateMachineError as e:
            print(f"[MissionState] Error: {e}")
            return False

    def pause(self) -> bool:
        """A helper method to transition to the PAUSED state."""
        return self.transition(MissionStateEnum.PAUSED)

    def resume(self) -> bool:
        """
        A helper method to resume from PAUSED to the previous state.
        """
        if self.current != MissionStateEnum.PAUSED or self._previous_state is None:
            print(f"[MissionState] Cannot resume: Not paused or no previous state recorded.")
            return False
        
        # Resume to the state we were in before pausing
        return self.transition(self._previous_state)

    @property
    def current(self) -> MissionStateEnum:
        """Returns the current state."""
        return self._fsm.current

    @property
    def history(self) -> List[StateLogEntry]:
        """Returns a copy of the state history log."""
        return list(self._state_history)

    def __str__(self) -> str:
        return f"MissionState(current={self.current.name})"