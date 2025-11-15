"""
Defines the TrackedTarget object used by the Hub to manage
the lifecycle of a detected target.
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Dict, Set

# Import the drone's position definition
from drone.core.utils.position import LocalPosition

class TargetStatus(Enum):
    """
    Defines the lifecycle of a target.
    """
    POTENTIAL = auto()    # First detection by a single drone
    CONFIRMED = auto()    # Corroborated by >1 drone, or by GCS
    DISREGARDED = auto()  # Marked as a false positive
    ENGAGED = auto()      # A payload drone is en route
    CLOSED = auto()       # Mission complete for this target

@dataclass
class TrackedTarget:
    """
    A hub-level object to track a target's state, position,
    and history.
    """
    track_id: str
    class_label: str
    status: TargetStatus = TargetStatus.POTENTIAL
    
    first_seen: float = field(default_factory=time.monotonic)
    last_seen: float = field(default_factory=time.monotonic)
    
    # Store a history of geolocated positions
    position_history: List[LocalPosition] = field(default_factory=list)
    
    # Store which drones have seen this target
    supporting_drones: Set[str] = field(default_factory=set)
    
    # Simple rule: >1 drone confirms the target
    CONFIRMATION_THRESHOLD: int = 2

    @property
    def latest_position(self) -> LocalPosition:
        """Returns the most recent known position of the target."""
        return self.position_history[-1] if self.position_history else LocalPosition()
        
    def add_detection(self, world_pos: LocalPosition, drone_id: str, timestamp: float):
        """
        Adds a new detection to this track.
        This is the core logic for updating the target.
        """
        self.last_seen = timestamp
        self.position_history.append(world_pos)
        self.supporting_drones.add(drone_id)
        
        # --- Consensus Logic ---
        # If this is still a potential target and now has enough
        # unique drones, confirm it.
        if self.status == TargetStatus.POTENTIAL and \
           len(self.supporting_drones) >= self.CONFIRMATION_THRESHOLD:
            
            self.status = TargetStatus.CONFIRMED