import cv2
import numpy as np
from .base import BaseDetector, DetectionResult

class ThermalEdgeDetector(BaseDetector):
    def process_frame(self, frame_data):
        # Normalize to 0-255 for OpenCV
        norm_frame = cv2.normalize(frame_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        # Apply Canny Edge Detection
        edges = cv2.Canny(norm_frame, 50, 150)
        
        # Find Contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 50: # Filter noise
                x, y, w, h = cv2.boundingRect(cnt)
                detections.append(DetectionResult(
                    label="thermal_shape",
                    confidence=0.7,
                    bbox=(x, y, w, h),
                    metadata={"method": "edge"}
                ))
        return detections