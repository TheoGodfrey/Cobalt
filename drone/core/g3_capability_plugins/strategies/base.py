"""
Component 7: Strategy (Base Interface)
Defines the abstract base class for all motion planning and
path generation plugins.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Protocol

# --- Data Structures ---
# Placeholder for a waypoint. Would be in a shared 'utils' module.
@dataclass
class Waypoint:
    """A 3D navigation target."""
    x: float
    y: float
    z: float # Altitude (e.g., -50 for 50m)
    yaw: float = 0.0

class VehicleState(Protocol):
    """Protocol for the VehicleState component (G4)."""
    async def get_position(self) -> Waypoint:
        ...
    # ... other state properties like battery, wind, etc.

# --- Base Class ---

class BaseStrategy(ABC):
    """
    Abstract interface for all motion strategies.
    Behaviors (G2) will call `next_waypoint()` to get
    their next navigation target.
    """
    
    def __init__(self, vehicle_state: VehicleState, config: Any = None):
        """
        Inject the vehicle state for context-aware planning.
        
        Args:
            vehicle_state: The VehicleState (G4) object.
            config: Optional strategy-specific configuration
                    (e.g., search area bounds).
        """
        self.vehicle_state = vehicle_state
        self.config = config

    @abstractmethod
    async def next_waypoint(self) -> Waypoint:
        """
        Computes and returns the next waypoint.
        
        This is the core method for all strategies.
        - A search strategy would calculate the next search point.
        - A delivery strategy would return the drop-off point.
        - An RTH strategy would return the home position.
        """
        pass
