"""
Component 8: Actuator (Base Interface)
Defines the abstract base class for all payload and
action-control plugins.
"""

from abc import ABC, abstractmethod
from typing import Protocol, Any

class ActuatorHardware(Protocol):
    """Protocol for a generic actuator (e.g., servo, dropper)."""
    async def set_angle(self, angle: float): ...
    async def release(self): ...

class BaseActuator(ABC):
    """
    Abstract interface for all physical action actuators.
    Behaviors (G2) will be given an instance of a class that
    implements this interface.
    """
    
    def __init__(self, hardware: ActuatorHardware, config: Any = None):
        """
        Inject the required hardware interface.
        
        Args:
            hardware: An object adhering to the ActuatorHardware protocol,
                      provided by the HAL (G4).
            config: Optional actuator-specific configuration.
        """
        self.hardware = hardware
        self.config = config

    @abstractmethod
    async def execute(self) -> bool:
        """
        Executes the actuator's action (e.g., drop payload).
        
        Returns:
            True on success, False on failure.
        """
        pass
