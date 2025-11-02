from abc import ABC, abstractmethod
import asyncio
from .drone import Drone

class BaseActuator(ABC):
    """L8: Abstract interface for any non-flight physical action."""
    
    @abstractmethod
    async def perform_action(self, drone: Drone, params: dict):
        """Execute the defined action (e.g., drop payload, flash light)."""
        pass

class SimplePayloadDropActuator(BaseActuator):
    """Simulates dropping a simple payload and flashing lights."""
    
    def __init__(self):
        self.name = "simple_drop"
    
    async def perform_action(self, drone: Drone, params: dict):
        """Flashes lights and simulates release."""
        duration = params.get("duration_s", 2.0)
        
        await drone.set_led("red")
        await asyncio.sleep(0.5)
        await drone.set_led("green")
        await asyncio.sleep(0.5)
        
        print(f"[Actuator] Payload {drone.id} released. Duration: {duration}s.")
        await asyncio.sleep(duration)
        await drone.set_led("off")
        
        return True

class LedSignalActuator(BaseActuator):
    """Flashes an SOS pattern."""
    
    def __init__(self):
        self.name = "led_signal"
    
    async def perform_action(self, drone: Drone, params: dict):
        color = params.get("color", "blue")
        repeats = params.get("repeats", 3)
        print(f"[Actuator] Signaling SOS pattern (x{repeats}).")
        
        for _ in range(repeats):
            await drone.set_led(color)
            await asyncio.sleep(0.1)
            await drone.set_led("off")
            await asyncio.sleep(0.1)
            await drone.set_led(color)
            await asyncio.sleep(0.1)
            await drone.set_led("off")
            await asyncio.sleep(0.3)
        await drone.set_led("off")

# --- Factory ---
_ACTUATORS = {
    "simple_drop": SimplePayloadDropActuator(),
    "led_signal": LedSignalActuator()
}

def get_actuator(name: str) -> BaseActuator:
    """Factory to retrieve an Actuator instance."""
    if name not in _ACTUATORS:
        raise ValueError(f"Unknown actuator: {name}")
    return _ACTUATORS[name]