"""
Visual camera detector for person confirmation
"""
import numpy as np
from typing import List
from ..cameras.base import VisualFrame, Detection

class VisualDetector:
    """Visual-based person detection for confirmation"""
    
    def __init__(self, config: dict):
        """
        Initialize visual detector
        
        Config parameters:
            - use_color: Use color detection (default: True)
            - use_motion: Use motion detection (default: True)
            - min_confidence: Minimum confidence threshold (default: 0.6)
        """
        self.use_color = config.get('use_color', True)
        self.use_motion = config.get('use_motion', True)
        self.min_confidence = config.get('min_confidence', 0.6)
        
        self.previous_frame = None
    
    def detect(self, frame: VisualFrame) -> List[Detection]:
        """Detect persons in visual frame"""
        
        detections = []
        
        # Color-based detection (skin tone detection)
        if self.use_color:
            color_detections = self._detect_by_color(frame)
            detections.extend(color_detections)
        
        # Motion-based detection
        if self.use_motion and self.previous_frame is not None:
            motion_detections = self._detect_by_motion(frame, self.previous_frame)
            detections.extend(motion_detections)
        
        self.previous_frame = frame
        
        # Filter by confidence
        detections = [d for d in detections if d.confidence >= self.min_confidence]
        
        return detections
    
    def _detect_by_color(self, frame: VisualFrame) -> List[Detection]:
        """Detect by skin color in RGB image"""
        image = frame.image
        
        # Simple skin color detection in RGB
        # Skin tone typically: R > 95, G > 40, B > 20, R > G, R > B, |R-G| > 15
        r = image[:, :, 0]
        g = image[:, :, 1]
        b = image[:, :, 2]
        
        skin_mask = (
            (r > 95) & (g > 40) & (b > 20) &
            (r > g) & (r > b) &
            (np.abs(r.astype(int) - g.astype(int)) > 15)
        )
        
        # Find blobs in skin mask
        from scipy import ndimage
        labeled_array, num_features = ndimage.label(skin_mask)
        
        detections = []
        
        for label_id in range(1, num_features + 1):
            blob_mask = labeled_array == label_id
            blob_size = np.sum(blob_mask)
            
            # Filter by size (person's head should be significant)
            if blob_size < 100 or blob_size > 5000:
                continue
            
            # Get centroid
            y_coords, x_coords = np.where(blob_mask)
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            
            # Calculate confidence based on size and color match quality
            confidence = min(1.0, blob_size / 1000.0) * 0.7
            
            detection = Detection(
                position_image=(center_x, center_y),
                position_world=None,
                confidence=confidence,
                is_person=True,
                source='visual_color',
                metadata={
                    'detection_method': 'color',
                    'blob_size': blob_size
                }
            )
            detections.append(detection)
        
        return detections
    
    def _detect_by_motion(self, current_frame: VisualFrame, previous_frame: VisualFrame) -> List[Detection]:
        """Detect by motion between frames"""
        
        # Simple frame difference
        diff = np.abs(current_frame.image.astype(int) - previous_frame.image.astype(int))
        motion_magnitude = np.sum(diff, axis=2)  # Sum across RGB channels
        
        # Threshold for significant motion
        motion_mask = motion_magnitude > 100
        
        from scipy import ndimage
        labeled_array, num_features = ndimage.label(motion_mask)
        
        detections = []
        
        for label_id in range(1, num_features + 1):
            blob_mask = labeled_array == label_id
            blob_size = np.sum(blob_mask)
            
            if blob_size < 50:  # Filter small motion
                continue
            
            y_coords, x_coords = np.where(blob_mask)
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            
            confidence = 0.5  # Motion alone is less reliable
            
            detection = Detection(
                position_image=(center_x, center_y),
                position_world=None,
                confidence=confidence,
                is_person=False,  # Motion could be anything
                source='visual_motion',
                metadata={
                    'detection_method': 'motion',
                    'motion_magnitude': float(np.mean(motion_magnitude[blob_mask]))
                }
            )
            detections.append(detection)
        
        return detections