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
# --- NEW Imports for SimulatedCamera FIX ---
import numpy as np 
# --- End NEW ---

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
        Returns a simulated frame (now a NumPy array) to satisfy the detector.
        
        The detector expects an image that can be copied and converted to grayscale.
        We return a simple 640x480 BGR image.
        """
        if not self._is_streaming:
            await self.start_streaming()
            
        await asyncio.sleep(1/30) # Simulate 30fps
        self._frame_count += 1
        
        # --- FIX: Return a dummy NumPy array (BGR image) ---
        # 480 rows (height), 640 columns (width), 3 color channels (BGR)
        # Create a solid black image to start
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Simulate a small white square (high heat signature)
        # BGR = [255, 255, 255] is white.
        if self._frame_count % 30 == 0:
            frame[200:220, 300:320] = [255, 255, 255] 
            
        return frame
        # --- End FIX ---