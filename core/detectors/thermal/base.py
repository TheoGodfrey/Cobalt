from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np

@dataclass
class DetectionResult:
    """Standardized output for any detector."""
    label: str
    confidence: float
    bbox: tuple  # (x, y, w, h)
    metadata: dict # Extensible for temp, stats, etc.

class BaseDetector(ABC):
    @abstractmethod
    def process_frame(self, frame_data: np.ndarray) -> list[DetectionResult]:
        """
        Input: Raw sensor frame (thermal or visual).
        Output: List of detections.
        """
        pass