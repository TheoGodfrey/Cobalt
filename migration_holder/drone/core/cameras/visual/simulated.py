"""
Simulated visual/RGB camera for testing
"""
import numpy as np
import time
import random
from ..base import BaseCamera, VisualFrame

class SimulatedVisualCamera(BaseCamera):
    """Simulated RGB camera with synthetic MOB scenarios"""
    
    def __init__(self, resolution=(640, 480), intrinsics=None): # <-- FIX: Added intrinsics=None
        """
        Initialize simulated visual camera
        
        Args:
            resolution: Camera resolution (width, height)
        """
        self.resolution = resolution
        self.connected = False
        self.frame_count = 0
        
        # Colors
        self.water_color = np.array([30, 80, 140])  # Blue water
        self.person_color = np.array([200, 150, 120])  # Skin tone
        self.person_present = False
        self.person_position = None
    
    def connect(self) -> bool:
        """Connect to simulated camera"""
        self.connected = True
        print(f"[Visual Sim] Connected - {self.resolution[0]}x{self.resolution[1]}")
        return True
    
    def capture(self) -> VisualFrame:
        """Generate synthetic RGB frame"""
        if not self.connected:
            raise RuntimeError("Camera not connected")
        
        self.frame_count += 1
        width, height = self.resolution
        
        # Create water background with waves
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Water with noise and waves
        for c in range(3):
            frame[:, :, c] = self.water_color[c] + np.random.randint(-20, 20, (height, width))
        
        # Add wave patterns
        for y in range(height):
            wave = int(5 * np.sin(y / 20.0 + self.frame_count / 10.0))
            frame[y, :, :] = np.roll(frame[y, :, :], wave, axis=0)
        
        # Randomly add person (synchronized with thermal via external flag)
        if random.random() < 0.3:  # Should sync with thermal detection
            self.person_present = True
            self._add_person(frame)
        else:
            self.person_present = False
        
        return VisualFrame(
            image=frame,
            resolution=self.resolution,
            timestamp=time.time(),
            frame_number=self.frame_count,
            metadata={
                'person_present': self.person_present,
                'person_position': self.person_position
            }
        )
    
    def _add_person(self, frame: np.ndarray):
        """Add person to visual frame"""
        height, width = frame.shape[:2]
        
        # Random position
        x = random.randint(width // 4, 3 * width // 4)
        y = random.randint(height // 4, 3 * height // 4)
        self.person_position = (x, y)
        
        # Person appears as head/shoulders
        head_radius = random.randint(15, 25)
        
        # Draw head (circular)
        for dy in range(-head_radius, head_radius):
            for dx in range(-head_radius, head_radius):
                if dx*dx + dy*dy < head_radius*head_radius:
                    px = x + dx
                    py = y + dy
                    
                    if 0 <= px < width and 0 <= py < height:
                        # Add skin color with some variation
                        frame[py, px] = self.person_color + np.random.randint(-10, 10, 3)
        
        # Add some splashing/white water around person
        splash_radius = head_radius + 10
        for _ in range(20):
            angle = random.uniform(0, 2 * np.pi)
            dist = random.uniform(head_radius, splash_radius)
            sx = int(x + dist * np.cos(angle))
            sy = int(y + dist * np.sin(angle))
            
            if 0 <= sx < width and 0 <= sy < height:
                frame[sy, sx] = [255, 255, 255]  # White splash
    
    def get_resolution(self) -> tuple:
        """Get camera resolution"""
        return self.resolution
    
    def disconnect(self):
        """Disconnect from camera"""
        self.connected = False
        print("[Visual Sim] Disconnected")