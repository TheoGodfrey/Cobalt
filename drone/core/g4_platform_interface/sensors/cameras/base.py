"""
Sensor Interface: Base Camera
Defines the abstract interface for a camera system.

This interface is "implemented" by the HAL (G4) and "consumed"
by Detector plugins (G3).
"""

import asyncio
import time
from abc import ABC, abstractmethod
from typing import Any

class BaseCamera(ABC):
    """
    Abstract interface for a camera.
    """
    
    @abstractmethod
    async def get_frame(self) -> Any:
        """
        Get the latest frame from the camera.
        
        Returns:
            A camera frame (e.g., numpy array, PIL Image)
        """
        pass

    @abstractmethod
    async def start_streaming(self):
        """Start the camera feed."""
        pass
        
    @abstractmethod
    async def stop_streaming(self):
        """Stop the camera feed."""
        pass

class SimulatedCamera(BaseCamera):
    """
    Simulated camera implementation.
    This would be provided by the SimulatedController (HAL).
    """
    def __init__(self, camera_id: int):
        self._camera_id = camera_id
        self._is_streaming = False
        self._frame_count = 0
        print(f"[SimulatedCamera {self._camera_id}] Initialized.")

    async def start_streaming(self):
        self._is_streaming = True
        print(f"[SimulatedCamera {self._camera_id}] Streaming started.")

    async def stop_streaming(self):
        self._is_streaming = False
        print(f"[SimulatedCamera {self._camera_id}] Streaming stopped.")

    async def get_frame(self) -> Any:
        """
        Returns a simulated frame (just a string with a counter).
        In a real system, this would be a numpy array.
        """
        if not self._is_streaming:
            await self.start_streaming()
            
        await asyncio.sleep(1/30) # Simulate 30fps
        self._frame_count += 1
        return f"SimulatedFrame_{self._frame_count}_Cam{self._camera_id}_{time.monotonic()}"
