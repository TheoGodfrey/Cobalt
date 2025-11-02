import cv2
import numpy as np
from scipy import ndimage
from typing import List, Tuple, Dict, Any
import time

def statistical_detection(thermal_image: np.ndarray) -> Dict[str, Any]:
    """Statistical method - works for small bodies relative to frame"""
    filtered = cv2.GaussianBlur(thermal_image, (5, 5), 0)
    water_median = np.median(filtered)
    water_std = np.std(filtered)
    
    if water_std < 1.0:
        return {"bounding_boxes": [], "confidence": 0.0, "method": "statistical", "color": (255, 0, 0)}  # Red
    
    hot_threshold = water_median + 3 * water_std
    hot_mask = (filtered > hot_threshold).astype(np.uint8)
    
    kernel = np.ones((5,5), np.uint8)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
    
    result = _analyze_blobs(hot_mask, "statistical")
    result["color"] = (255, 0, 0)  # Red
    return result

def edge_based_detection(thermal_image: np.ndarray) -> Dict[str, Any]:
    """Edge-based detection - works for bodies of any size"""
    normalized = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    
    grad_x = cv2.Sobel(normalized, cv2.CV_64F, 1, 0, ksize=5)
    grad_y = cv2.Sobel(normalized, cv2.CV_64F, 0, 1, ksize=5)
    gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
    
    edge_threshold = np.percentile(gradient_magnitude, 85)
    edge_mask = (gradient_magnitude > edge_threshold).astype(np.uint8)
    
    contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    bounding_boxes = []
    total_confidence = 0
    valid_detections = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if 500 < area < 10000:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            
            if 0.5 < aspect_ratio < 3.0:
                bounding_boxes.append((x, y, w, h))
                
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                solidity = area / hull_area if hull_area > 0 else 0
                confidence = min(solidity * 0.7 + (area / 2000) * 0.3, 1.0)
                total_confidence += confidence
                valid_detections += 1
    
    avg_confidence = total_confidence / max(valid_detections, 1)
    
    return {
        "bounding_boxes": bounding_boxes, 
        "confidence": avg_confidence, 
        "method": "edge_based",
        "color": (0, 255, 0)  # Green
    }

def absolute_threshold_detection(thermal_image: np.ndarray, 
                               min_temp: float = 30.0, 
                               max_temp: float = 40.0) -> Dict[str, Any]:
    """Absolute temperature thresholding - uses known physiological ranges"""
    body_mask = ((thermal_image >= min_temp) & 
                (thermal_image <= max_temp)).astype(np.uint8)
    
    kernel = np.ones((3,3), np.uint8)
    body_mask = cv2.morphologyEx(body_mask, cv2.MORPH_CLOSE, kernel)
    body_mask = cv2.morphologyEx(body_mask, cv2.MORPH_OPEN, kernel)
    
    result = _analyze_blobs(body_mask, "absolute_threshold")
    result["color"] = (0, 0, 255)  # Blue
    return result

def background_subtraction_detection(thermal_image: np.ndarray, 
                                   background_model: np.ndarray = None,
                                   difference_threshold: float = 2.0) -> Dict[str, Any]:
    """Background subtraction - works with only static cameras/known background. Will likely be unused"""
    if background_model is None:
        return {
            "bounding_boxes": [], 
            "confidence": 0.0, 
            "method": "background_subtraction",
            "color": (255, 255, 0)  # Cyan
        }
    
    difference = np.abs(thermal_image - background_model)
    diff_mask = (difference > difference_threshold).astype(np.uint8)
    
    kernel = np.ones((5,5), np.uint8)
    diff_mask = cv2.morphologyEx(diff_mask, cv2.MORPH_CLOSE, kernel)
    
    result = _analyze_blobs(diff_mask, "background_subtraction")
    result["color"] = (255, 255, 0)  # Cyan
    return result

def _analyze_blobs(mask: np.ndarray, method_name: str) -> Dict[str, Any]:
    """Helper function to analyze connected components in a mask"""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    
    bounding_boxes = []
    total_confidence = 0
    valid_detections = 0
    
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        width = stats[i, cv2.CC_STAT_WIDTH]
        height = stats[i, cv2.CC_STAT_HEIGHT]
        aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 0
        
        min_area, max_area = 100, 5000
        min_ar, max_ar = 0.5, 3.0
        
        if min_area < area < max_area and min_ar < aspect_ratio < max_ar:
            x, y, w, h = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                         stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
            bounding_boxes.append((x, y, w, h))
            
            size_conf = 1.0 - abs(area - 1000) / 1000
            shape_conf = 1.0 - min(abs(aspect_ratio - 1.5) / 2.0, 1.0)
            confidence = (size_conf + shape_conf) / 2
            total_confidence += max(confidence, 0)
            valid_detections += 1
    
    avg_confidence = total_confidence / max(valid_detections, 1)
    return {"bounding_boxes": bounding_boxes, "confidence": avg_confidence, "method": method_name}

def combine_detections(thermal_image: np.ndarray, 
                      use_background_subtraction: bool = False,
                      background_model: np.ndarray = None) -> Dict[str, Any]:
    """
    Combine all detection methods for robust body detection
    Returns results from each method separately for debugging
    """
    # Run all detection methods
    results = {}
    
    results["statistical"] = statistical_detection(thermal_image)
    results["edge_based"] = edge_based_detection(thermal_image)
    results["absolute_threshold"] = absolute_threshold_detection(thermal_image)
    
    if use_background_subtraction and background_model is not None:
        results["background_subtraction"] = background_subtraction_detection(thermal_image, background_model)
    
    # Calculate overall confidence (average of all methods)
    confidences = [result["confidence"] for result in results.values()]
    overall_confidence = sum(confidences) / len(confidences)
    
    return {
        "method_results": results,
        "overall_confidence": overall_confidence,
        "methods_used": list(results.keys())
    }

def visualize_detections(thermal_image: np.ndarray, 
                        detection_results: Dict[str, Any],
                        display_confidence: bool = True) -> np.ndarray:
    """
    Create a visualization with different colors for each detection method
    """
    # Convert thermal image to BGR for color drawing
    if len(thermal_image.shape) == 2:
        visual = cv2.normalize(thermal_image, None, 0, 255, cv2.NORM_MINMAX)
        visual = visual.astype(np.uint8)
        visual = cv2.cvtColor(visual, cv2.COLOR_GRAY2BGR)
    else:
        visual = thermal_image.copy()
    
    # Draw detections from each method
    method_results = detection_results["method_results"]
    
    for method_name, result in method_results.items():
        color = result.get("color", (255, 255, 255))  # Default to white if no color
        boxes = result["bounding_boxes"]
        confidence = result["confidence"]
        
        for i, (x, y, w, h) in enumerate(boxes):
            # Draw bounding box
            cv2.rectangle(visual, (x, y), (x + w, y + h), color, 2)
            
            # Draw label with method name and confidence
            if display_confidence:
                label = f"{method_name}: {confidence:.2f}"
            else:
                label = method_name
                
            cv2.putText(visual, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # Add overall confidence to the image
    overall_conf = detection_results["overall_confidence"]
    cv2.putText(visual, f"Overall Confidence: {overall_conf:.2f}", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Add legend
    y_offset = 60
    for method_name, result in method_results.items():
        color = result.get("color", (255, 255, 255))
        confidence = result["confidence"]
        boxes_count = len(result["bounding_boxes"])
        
        legend_text = f"{method_name}: {boxes_count} boxes, conf: {confidence:.2f}"
        cv2.putText(visual, legend_text, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y_offset += 20
    
    return visual

# Example usage with visualization
if __name__ == "__main__":
    # Create a simulated thermal image
    thermal_image = np.random.normal(20, 2, (480, 640)).astype(np.float32)
    
    # Add simulated warm bodies at different sizes
    thermal_image[200:250, 300:350] = 35  # Small body
    thermal_image[100:200, 100:200] = 36  # Medium body  
    thermal_image[300:450, 400:600] = 34  # Large body
    
    # Run combined detection
    results = combine_detections(thermal_image)
    
    # Create visualization
    visualization = visualize_detections(thermal_image, results)
    
    # Display results
    print(f"Overall confidence: {results['overall_confidence']:.2f}")
    print(f"Methods used: {results['methods_used']}")
    
    for method_name, method_result in results['method_results'].items():
        boxes = method_result['bounding_boxes']
        confidence = method_result['confidence']
        print(f"{method_name}: {len(boxes)} boxes, confidence: {confidence:.2f}")
    
    # Show the visualization
    cv2.imshow('Thermal Detection Debug View', visualization)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save the visualization
    cv2.imwrite('thermal_detection_debug.png', visualization)
