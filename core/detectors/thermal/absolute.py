import numpy as np
import cv2
from .base import BaseDetector, DetectionResult

class AbsoluteThermalDetector(BaseDetector):
    """
    Detects regions with a temperature above a fixed threshold.
    Assumes the input frame is temperature data (or can be treated as such).
    """
    def __init__(self, threshold_temp=38.0):
        self.threshold = threshold_temp

    def process_frame(self, frame_data):
        # Ensure we have a numpy array
        if not isinstance(frame_data, np.ndarray):
            return []

        # Create a binary mask where pixels > threshold
        # For simulation/MVP, we assume frame_data values map somewhat to temperature
        # or are pre-normalized.
        mask = frame_data > self.threshold
        mask = mask.astype(np.uint8) * 255

        # Find contours (blobs) of hot areas
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for cnt in contours:
            # Filter small noise
            if cv2.contourArea(cnt) < 10: 
                continue
                
            x, y, w, h = cv2.boundingRect(cnt)
            
            # Calculate approximate confidence based on "heat" intensity or size
            # For MVP, we just say 0.9 if it passed the threshold
            confidence = 0.9 

            detections.append(DetectionResult(
                label="thermal_hotspot",
                confidence=confidence,
                bbox=(x, y, w, h),
                metadata={"threshold": self.threshold}
            ))
            
        return detections