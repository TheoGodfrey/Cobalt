"""
Component 6: Detector (Thermal Implementation)
A concrete implementation of BaseDetector for thermal sensing.
"""

import asyncio
import time
import random
from typing import Optional, Any
from .base import BaseDetector, Detection, Camera

class ThermalDetector(BaseDetector):
    """
    Simulates a thermal detector.
    
    In a real system, this class would contain complex computer vision
    logic (e.g., running a model on the camera frame).
    """
    
    def __init__(self, camera: Camera, threshold: float = 0.9):
        super().__init__(camera)
        self.confidence_threshold = threshold
        print(f"[ThermalDetector] Initialized with threshold {self.confidence_threshold}")

    async def detect(self) -> Optional[Detection]:
        """
        Simulates getting a thermal frame and processing it.
        """
        # 1. Get frame (simulated)
        frame = await self.camera.get_frame()
        # print(f"[ThermalDetector] Processing frame {frame}...")
        
        # 2. Process the frame (simulated)
        # Simulate processing time
        await asyncio.sleep(0.5) 
        
        # Simulate a random detection
        if random.random() > 0.9: # 10% chance to detect
            print("[ThermalDetector] *** Potential Target Sighted! ***")
            return Detection(
                timestamp=time.monotonic(),
                class_label="human",
                confidence=0.95,
                position=(random.randint(100, 500), random.randint(100, 500)),
                temperature=37.0
            )
        
        # 3. No detection
        return None
