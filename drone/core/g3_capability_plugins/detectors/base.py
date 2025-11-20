"""
Component 6: Detector (Base Interface)
Defines the abstract base class for all Detector plugins.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Any, Protocol

# --- Data Structures ---
# These would be defined in a shared 'utils' module in a real system
# to avoid circular dependencies and allow type hinting.

class Camera(Protocol):
    """Protocol for a camera hardware interface."""
    async def get_frame(self) -> Any: # e.g., a numpy array
        ...

@dataclass
class Detection:
    """A standardized data structure for a detection result."""
    timestamp: float
    class_label: str               # "human", "vessel", "debris"
    confidence: float              # 0.0 - 1.0
    
    # Position can be in different coordinate systems
    # (e.g., pixel_coords, world_coords)
    position: Any 
    
    # Optional thermal data
    temperature: Optional[float] = None
    
    # Optional tracking ID
    track_id: Optional[int] = None

# --- Base Class ---

class BaseDetector(ABC):
    """
    Abstract interface for all sensing and detection algorithms.
    Behaviors (G2) will be given an instance of a class that
    implements this interface.
    """
    
    def __init__(self, camera: Camera):
        """
        Inject the required hardware (e.g., a camera).
        
        Args:
            camera: An object adhering to the Camera protocol,
                    provided by the HAL (G4).
        """
        self.camera = camera

    @abstractmethod
    async def detect(self) -> Optional[Detection]:
        """
        Performs a detection cycle.
        
        This method should:
        1. Get a frame from self.camera.
        2. Process the frame.
        3. Return a Detection object if a target is found,
           otherwise return None.
        """
        pass
