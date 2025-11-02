"""
Component 8: Actuator (Simple Payload Dropper)
A concrete implementation of BaseActuator for a simple release mechanism.
"""

import asyncio
from .base import BaseActuator, ActuatorHardware

class PayloadDropper(BaseActuator):
    """
    Implements a simple payload drop by calling 'release'
    on the injected hardware.
    """
    
    def __init__(self, hardware: ActuatorHardware, config: dict = None):
        super().__init__(hardware, config)
        self.release_time = config.get("release_time_sec", 2) if config else 2
        print("[PayloadDropper] Initialized.")

    async def execute(self) -> bool:
        """
        Executes the payload drop.
        """
        try:
            print(f"[PayloadDropper] Activating release mechanism for {self.release_time}s...")
            await self.hardware.release()
            await asyncio.sleep(self.release_time) # Hold servo
            print("[PayloadDropper] Release complete.")
            return True
        except Exception as e:
            print(f"[PayloadDropper] ERROR: Release failed: {e}")
            return False
