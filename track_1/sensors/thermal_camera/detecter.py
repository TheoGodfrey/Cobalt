import cv2
import numpy as np

def method_1_threshold(thermal_frame, temp_threshold=30):
    """
    Detects human body heat signatures above water temperature
    """
    # Normalize thermal data for consistent processing
    normalized_frame = cv2.normalize(thermal_frame.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
    normalized_frame = normalized_frame.astype(np.uint8)
    
    # Apply threshold to find warmer regions (potential human body)
    _, thresholded = cv2.threshold(normalized_frame, temp_threshold, 255, cv2.THRESH_BINARY)
    
    # Find contours of warm regions
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 50:  # Minimum area to avoid noise
            x, y, w, h = cv2.boundingRect(contour)
            detections.append((x, y, w, h, area))
    
    return detections, thresholded