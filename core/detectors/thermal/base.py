
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Tuple, Any

@dataclass
class DetectionResult:
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int] # (x, y, w, h)
    metadata: Any = field(default_factory=dict)
    
class BaseDetector(ABC):
    @abstractmethod
    def process_frame(self, frame_data): 
        pass