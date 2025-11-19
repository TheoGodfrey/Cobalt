import numpy as np
from .base import BaseDetector, DetectionResult

class StatisticalAnomalyDetector(BaseDetector):
    def __init__(self, sigma=3.0):
        self.sigma = sigma

    def process_frame(self, frame_data):
        mean = np.mean(frame_data)
        std = np.std(frame_data)
        
        # Detect outliers (Z-score > sigma)
        anomaly_mask = (frame_data - mean) > (self.sigma * std)
        
        if not np.any(anomaly_mask):
            return []

        # (Blobbing logic similar to absolute detector would go here)
        return [DetectionResult(
            label="thermal_anomaly",
            confidence=0.85,
            bbox=(0,0,0,0), # Placeholder
            metadata={"z_score": self.sigma}
        )]