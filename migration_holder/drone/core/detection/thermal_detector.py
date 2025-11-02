"""
Temperature threshold-based thermal detection
"""
import numpy as np
from typing import List
from ..cameras.base import ThermalFrame, Detection

class ThermalDetector:
    """Simple threshold-based thermal detection"""
    
    def __init__(self, config: dict):
        """
        Initialize threshold detector
        
        Config parameters:
            - temp_threshold: Temperature above water (default: 10.0°C)
            - min_area: Minimum detection area in pixels (default: 50)
            - max_area: Maximum detection area in pixels (default: 500)
            - min_confidence: Minimum confidence to report (default: 0.5)
        """
        self.temp_threshold = config.get('temp_threshold', 10.0)
        self.min_area = config.get('min_area', 50)
        self.max_area = config.get('max_area', 500)
        self.min_confidence = config.get('min_confidence', 0.5)
        
        # Water temperature estimation (rolling average)
        self.estimated_water_temp = None
        self.water_temp_samples = []
        self.max_samples = 10
    
    def detect(self, frame: ThermalFrame) -> List[Detection]:
        """Detect heat signatures in thermal frame"""
        
        # Estimate water temperature from coldest regions
        self._update_water_temp_estimate(frame)
        
        if self.estimated_water_temp is None:
            return []
        
        # Find pixels above threshold
        threshold_temp = self.estimated_water_temp + self.temp_threshold
        hot_mask = frame.temperature_array > threshold_temp
        
        # Find connected components (blobs)
        detections = self._find_blobs(frame, hot_mask, threshold_temp)
        
        # Filter by confidence
        detections = [d for d in detections if d.confidence >= self.min_confidence]
        
        return detections
    
    def _update_water_temp_estimate(self, frame: ThermalFrame):
        """Estimate water temperature from frame"""
        # Use 10th percentile as water temperature estimate
        water_temp = np.percentile(frame.temperature_array, 10)
        
        self.water_temp_samples.append(water_temp)
        if len(self.water_temp_samples) > self.max_samples:
            self.water_temp_samples.pop(0)
        
        self.estimated_water_temp = np.mean(self.water_temp_samples)
    
    def _find_blobs(self, frame: ThermalFrame, mask: np.ndarray, threshold_temp: float) -> List[Detection]:
        """Find connected components in binary mask"""
        from scipy import ndimage
        
        # Label connected components
        labeled_array, num_features = ndimage.label(mask)
        
        detections = []
        
        for label_id in range(1, num_features + 1):
            # Get pixels for this blob
            blob_mask = labeled_array == label_id
            blob_pixels = np.sum(blob_mask)
            
            # Filter by size
            if blob_pixels < self.min_area or blob_pixels > self.max_area:
                continue
            
            # Get blob properties
            blob_temps = frame.temperature_array[blob_mask]
            peak_temp = np.max(blob_temps)
            avg_temp = np.mean(blob_temps)
            
            # Find centroid
            y_coords, x_coords = np.where(blob_mask)
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            
            # Bounding box
            x_min, x_max = int(np.min(x_coords)), int(np.max(x_coords))
            y_min, y_max = int(np.min(y_coords)), int(np.max(y_coords))
            width = x_max - x_min
            height = y_max - y_min
            
            # Calculate confidence score
            confidence = self._calculate_confidence(
                blob_pixels, peak_temp, avg_temp, width, height
            )
            
            # Classify as person based on features
            is_person = self._classify_person(
                blob_pixels, peak_temp, width, height, confidence
            )
            
            detection = Detection(
                position_image=(center_x, center_y),
                position_world=None,  # Will be calculated by mission controller
                confidence=confidence,
                is_person=is_person,
                source='thermal',
                metadata={
                    'temperature': peak_temp,
                    'avg_temperature': avg_temp,
                    'temp_above_water': peak_temp - self.estimated_water_temp,
                    'blob_size': blob_pixels,
                    'bounding_box': (x_min, y_min, width, height)
                }
            )
            detections.append(detection)
        
        return detections
    
    def _calculate_confidence(self, area: int, peak_temp: float, avg_temp: float, 
                             width: int, height: int) -> float:
        """Calculate detection confidence based on features"""
        # Size score (person should be 50-500 pixels)
        size_score = 1.0 if 50 <= area <= 500 else 0.5
        
        # Temperature score (higher temp = higher confidence)
        temp_score = min(1.0, (peak_temp - self.estimated_water_temp) / 25.0)
        
        # Shape score (person should be somewhat elongated)
        aspect_ratio = max(width, height) / (min(width, height) + 1)
        shape_score = 1.0 if 1.2 <= aspect_ratio <= 3.0 else 0.7
        
        # Combined confidence
        confidence = (size_score * 0.3 + temp_score * 0.5 + shape_score * 0.2)
        
        return min(1.0, confidence)
    
    def _classify_person(self, area: int, peak_temp: float, width: int, 
                         height: int, confidence: float) -> bool:
        """Classify if detection is likely a person"""
        # Simple heuristic: if confidence is high and temp is warm enough
        if confidence > 0.7 and peak_temp > (self.estimated_water_temp + 15):
            return True
        return False