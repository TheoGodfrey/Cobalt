"""
Simulated thermal camera for testing detection algorithms
"""
import numpy as np
import time
import random
from ..base import BaseCamera, ThermalFrame

class SimulatedThermalCamera(BaseCamera):
    """Simulated thermal camera with synthetic MOB scenarios"""
    
    def __init__(self, resolution=(160, 120), water_temp=15.0, ambient_temp=20.0, intrinsics=None): # <-- FIX: Added intrinsics=None
        """
        Initialize simulated thermal camera
        
        Args:
            resolution: Camera resolution (width, height)
            water_temp: Water temperature in Celsius
            ambient_temp: Ambient air temperature in Celsius
        """
        self.resolution = resolution
        self.water_temp = water_temp
        self.ambient_temp = ambient_temp
        self.connected = False
        self.emissivity = 0.98
        
        # Simulation parameters
        self.person_temp = 36.0  # Human body temperature
        self.person_present = False
        self.person_position = None
        self.detection_probability = 0.3  # 30% chance per frame
        
        # Frame counter for varied scenarios
        self.frame_count = 0
    
    def connect(self) -> bool:
        """Connect to simulated camera"""
        self.connected = True
        print(f"[Thermal Sim] Connected - {self.resolution[0]}x{self.resolution[1]}")
        return True
    
    def capture(self) -> ThermalFrame:
        """Generate synthetic thermal frame"""
        if not self.connected:
            raise RuntimeError("Camera not connected")
        
        self.frame_count += 1
        
        # Create base thermal image (water background)
        width, height = self.resolution
        frame = np.random.normal(self.water_temp, 0.5, (height, width))
        
        # Add noise and gradients for realism
        frame += np.random.normal(0, 0.3, (height, width))
        
        # Randomly place person in water
        if random.random() < self.detection_probability:
            self.person_present = True
            self._add_person_signature(frame)
        else:
            self.person_present = False
        
        # Add occasional false positives (debris, waves, etc.)
        if random.random() < 0.1:  # 10% chance
            self._add_false_positive(frame)
        
        min_temp = float(np.min(frame))
        max_temp = float(np.max(frame))
        
        return ThermalFrame(
            temperature_array=frame,
            timestamp=time.time(),
            frame_number=self.frame_count,
            min_temp=min_temp,
            max_temp=max_temp,
            resolution=self.resolution,
            metadata={
                'frame_number': self.frame_count,
                'person_present': self.person_present,
                'person_position': self.person_position
            }
        )
    
    def _add_person_signature(self, frame: np.ndarray):
        """Add realistic person heat signature to frame"""
        height, width = frame.shape
        
        # Random position
        x = random.randint(width // 4, 3 * width // 4)
        y = random.randint(height // 4, 3 * height // 4)
        self.person_position = (x, y)
        
        # Person appears as elongated blob (head/shoulders visible)
        # Size varies with distance (simulated altitude)
        size_x = random.randint(8, 15)  # Width in pixels
        size_y = random.randint(12, 20)  # Height in pixels
        
        # Create Gaussian heat signature
        for dy in range(-size_y//2, size_y//2):
            for dx in range(-size_x//2, size_x//2):
                px = x + dx
                py = y + dy
                
                if 0 <= px < width and 0 <= py < height:
                    # Distance from center
                    dist = np.sqrt(dx**2 + dy**2)
                    max_dist = np.sqrt((size_x/2)**2 + (size_y/2)**2)
                    
                    # Gaussian falloff
                    if dist < max_dist:
                        intensity = np.exp(-(dist**2) / (2 * (max_dist/2)**2))
                        temp_increase = (self.person_temp - self.water_temp) * intensity
                        frame[py, px] += temp_increase
    
    def _add_false_positive(self, frame: np.ndarray):
        """Add false positive (debris, wave, etc.)"""
        height, width = frame.shape
        
        x = random.randint(10, width - 10)
        y = random.randint(10, height - 10)
        
        # Smaller and cooler than person
        size = random.randint(3, 6)
        temp_increase = random.uniform(2, 5)  # Much less than person
        
        for dy in range(-size, size):
            for dx in range(-size, size):
                px = x + dx
                py = y + dy
                
                if 0 <= px < width and 0 <= py < height:
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist < size:
                        frame[py, px] += temp_increase * (1 - dist/size)
    
    def get_resolution(self) -> tuple:
        """Get camera resolution"""
        return self.resolution
    
    def set_emissivity(self, emissivity: float):
        """Set emissivity"""
        self.emissivity = max(0.0, min(1.0, emissivity))
    
    def disconnect(self):
        """Disconnect from camera"""
        self.connected = False
        print("[Thermal Sim] Disconnected")