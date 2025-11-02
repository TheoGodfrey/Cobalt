"""
Component 9: Hardware Abstraction Layer (HAL)
Unified abstract interface to vehicle hardware.

This is the ONLY component that should talk to hardware (or the simulator).
It provides a high-level API for other components (like Behaviors) to use.

It owns and manages:
- The VehicleState (Component 10)
- The low-level hardware interfaces (e.g., MAVLink connection)
- The sensor objects (e.g., BaseCamera)
- The actuator objects (e.g., ActuatorHardware)
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any

# Import component 10
from .vehicle_state import VehicleState

# Import protocols (interfaces) from G3/G4
from ..g3_capability_plugins.strategies.base import Waypoint
from .sensors.cameras.base import BaseCamera
from ..g3_capability_plugins.actuators.base import ActuatorHardware

# --- HAL Base Class ---

class BaseFlightController(ABC):
    """
    The abstract interface for the HAL.
    All high-level components (G2, G3) code against this interface.
    """
    
    # --- FIX for Bug #7 ---
    # Added config parameter to match the derived class's super() call
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initializes the HAL.
        
        Args:
            config: An optional configuration dictionary.
        """
        self.config = config if config is not None else {}
        self.vehicle_state = VehicleState()

    @abstractmethod
    async def connect(self):
        """Establish connection to the flight controller."""
        pass
        
    @abstractmethod
    async def arm(self):
        """Arm the vehicle."""
        pass
        
    @abstractmethod
    async def disarm(self):
        """Disarm the vehicle."""
        pass
        
    @abstractmethod
    async def goto(self, waypoint: Waypoint) -> bool:
        """
        Fly to a specific waypoint.
        Returns True on arrival, False on failure.
        """
        pass
        
    @abstractmethod
    async def land(self) -> bool:
        """Land at the current position."""
        pass
        
    @abstractmethod
    def get_camera(self, camera_id: int) -> BaseCamera:
        """Get an interface to a camera."""
        pass
        
    @abstractmethod
    def get_actuator_hardware(self, actuator_id: int) -> ActuatorHardware:
        """Get an interface to an actuator."""
        pass