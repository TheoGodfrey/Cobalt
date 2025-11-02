"""
Abstract camera interfaces for dual-camera system
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
import numpy as np
import time

@dataclass
class CameraFrame:
    """Base class for camera frame data"""
    timestamp: float
    frame_number: int
    metadata: dict

@dataclass
class ThermalFrame(CameraFrame):
    """Thermal camera frame data"""
    temperature_array: np.ndarray  # 2D array of temperatures (Celsius)
    min_temp: float
    max_temp: float
    resolution: tuple  # (width, height)

@dataclass
class VisualFrame(CameraFrame):
    """Visual/RGB camera frame data"""
    image: np.ndarray  # RGB image array (H, W, 3)
    resolution: tuple  # (width, height)

@dataclass
class Detection:
    """Unified detection from either camera"""
    position_image: tuple  # (x, y) in image coordinates
    position_world: Optional[object]  # 3D position if available
    confidence: float  # 0.0 to 1.0
    is_person: bool
    source: str  # 'thermal', 'visual', or 'fusion'
    metadata: dict

class BaseCamera(ABC):
    """Abstract base class for all cameras"""
    
    @abstractmethod
    def connect(self) -> bool:
        """Connect to camera"""
        pass
    
    @abstractmethod
    def capture(self) -> CameraFrame:
        """Capture a frame"""
        pass
    
    @abstractmethod
    def get_resolution(self) -> tuple:
        """Get camera resolution (width, height)"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Disconnect from camera"""
        pass