import cv2
import numpy as np
from core.detectors.thermal.base import BaseDetector, DetectionResult

class ColorBlobDetector(BaseDetector):
    """
    Detects specific colors (default: International Orange) in visual footage.
    """
    def __init__(self, color_range="orange", min_area=50):
        # HSV ranges for International Orange
        # Note: These need tuning based on your specific camera hardware
        if color_range == "orange":
            # OpenCV HSV is (0-179, 0-255, 0-255)
            # Orange is roughly Hue 10-25
            self.lower_hsv = np.array([5, 150, 150])
            self.upper_hsv = np.array([25, 255, 255])
        self.min_area = min_area

    def process_frame(self, frame_data):
        # 1. Preprocessing
        if frame_data is None: return []
        
        # Blur to remove high-frequency noise
        blurred = cv2.GaussianBlur(frame_data, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # 2. Thresholding
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # 3. Morphological Operations (Remove salt-and-pepper noise)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 4. Contour Detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Calculate confidence based on "solidity" (area / convex hull area)
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0
                
                # Calculate centroid for the bounding box
                results.append(DetectionResult(
                    label="visual_lifejacket",
                    confidence=0.6 + (0.3 * solidity), # Higher solidity = likely man-made
                    bbox=(x, y, w, h),
                    metadata={"color": "orange", "area": area}
                ))
                
        return results