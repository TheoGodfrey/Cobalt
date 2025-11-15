"""
Component 7: Strategy (Base Interface)
Defines the abstract base class for all motion planning and
path generation plugins.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Protocol

# --- FIX: Import the explicit LocalPosition ---
from ...utils.position import LocalPosition
# --- End of FIX ---

# --- Data Structures ---
# Waypoint is a *target* in local coordinates
@dataclass
class Waypoint:
    """A 3D navigation target."""
    x: float
    y: float
    z: float # Altitude (e.g., -50 for 50m)
    yaw: float = 0.0

class VehicleState(Protocol):
    """Protocol for the VehicleState component (G4)."""
    
    # --- FIX: Strategies use local position ---
    @property
    def position_local(self) -> LocalPosition:
        """Protocol for accessing the drone's local position."""
        ...
    # --- End of FIX ---
    
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