"""
Component 6: Detector (Thermal Implementation)
A concrete implementation of BaseDetector for thermal sensing.

This advanced implementation runs multiple strategies (Threshold,
Adaptive, Edge-Detection) in parallel to find thermal blobs,
and then uses a classifier to identify them.
"""

import asyncio
import time
import cv2
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Any, List, Dict, Tuple
from .base import BaseDetector, Detection, Camera

# --- BGR Colors for Debug Drawing ---
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (255, 0, 0)
COLOR_YELLOW = (0, 255, 255)

# --- Ported from old v_0.1 thermal_processing.py ---

class TargetType(Enum):
    """Enumeration of possible target classifications"""
    UNKNOWN = "unknown"
    PERSON = "person"
    BOAT = "boat"
    DEBRIS = "debris"

@dataclass
class ThermalBlob:
    """A contiguous hot region in thermal image"""
    center_x: int  # pixel coordinates
    center_y: int
    area: float  # pixels²
    mean_temp: float  # °C (approximated)
    max_temp: float  # °C (approximated)
    min_temp: float  # °C (approximated)
    aspect_ratio: float  # width/height (for shape analysis)
    bounding_box: Tuple[int, int, int, int] # (x, y, w, h)
    
    def __str__(self) -> str:
        return (f"ThermalBlob(center=({self.center_x}, {self.center_y}), "
                f"area={self.area:.0f}px², temp={self.mean_temp:.1f}°C)")

# --- End of Ported Code ---


class ThermalDetector(BaseDetector):
    """
    Detects thermal hotspots in a camera frame using multiple
    OpenCV strategies and a simple classifier.
    
    This implementation assumes a "white-hot" thermal camera feed.
    """
    
    # --- Ported from old v_0.1 thermal_camera.py ---
    class _ThermalClassifier:
        """
        A simple rule-based classifier for thermal blobs.
        This logic is ported from the old ThermalCamera class.
        """
        def __init__(self):
            # TODO: Altitude should be updated dynamically
            self.altitude_m = 30.0 
            
            # --- Tunable parameters for classification ---
            # Temperature range for a person
            self.PERSON_MIN_TEMP = 28.0
            self.PERSON_MAX_TEMP = 40.0
            # Temperature for a boat engine
            self.BOAT_ENGINE_TEMP = 50.0
            
            # Size (in pixels) estimates
            # These are highly dependent on camera/altitude
            self.PERSON_MIN_AREA = 20
            self.PERSON_MAX_AREA = 400
            self.BOAT_MIN_AREA = 500

        def set_altitude(self, altitude: float):
            """Updates altitude to adjust classification rules."""
            # In a real system, this would adjust area thresholds
            self.altitude_m = altitude

        def classify_blob(self, blob: ThermalBlob) -> Tuple[TargetType, float]:
            """
            Classifies a single ThermalBlob based on its properties.
            
            Returns:
                A tuple of (TargetType, confidence)
            """
            
            # --- Rule 1: Person ---
            # Check temperature range
            is_temp_human = (self.PERSON_MIN_TEMP <= blob.mean_temp <= self.PERSON_MAX_TEMP)
            # Check size range
            is_size_human = (self.PERSON_MIN_AREA <= blob.area <= self.PERSON_MAX_AREA)
            # Check shape (not too elongated)
            is_shape_human = (0.7 < blob.aspect_ratio < 1.5)
            
            if is_temp_human and is_size_human and is_shape_human:
                return (TargetType.PERSON, 0.95)
                
            # --- Rule 2: Boat ---
            # Check for a very hot spot (engine)
            is_temp_engine = (blob.max_temp > self.BOAT_ENGINE_TEMP)
            # Check for large size
            is_size_boat = (blob.area > self.BOAT_MIN_AREA)
            # Check for elongated shape
            is_shape_boat = (blob.aspect_ratio > 2.0 or blob.aspect_ratio < 0.5)

            if is_size_boat and (is_temp_engine or is_shape_boat):
                 return (TargetType.BOAT, 0.80)
                 
            # --- Rule 3: Unknown Hot Debris ---
            if blob.mean_temp > self.PERSON_MIN_TEMP:
                return (TargetType.DEBRIS, 0.5)

            return (TargetType.UNKNOWN, 0.0)
    # --- End of Ported Classifier ---

    
    def __init__(self, camera: Camera, config: dict = None):
        super().__init__(camera)
        config = config or {}
        
        # General config
        self.debug_mode = config.get("debug", False)
        self.min_area = config.get("min_area", 20) # Updated default
        self.confidence = config.get("confidence", 0.95) # This is now a fallback

        # Strategy 1: Simple Threshold
        self.st_config = config.get("strategy_simple_threshold", {})
        self.st_enabled = self.st_config.get("enabled", True)
        self.st_threshold = self.st_config.get("threshold", 220) # Lowered threshold

        # Strategy 2: Adaptive Threshold (Statistical)
        self.at_config = config.get("strategy_adaptive_threshold", {})
        self.at_enabled = self.at_config.get("enabled", True)
        self.at_block_size = self.at_config.get("block_size", 25) # Must be odd
        self.at_c_value = self.at_config.get("c_value", 10)

        # Strategy 3: Edge Detection (Canny)
        self.ed_config = config.get("strategy_edge_detection", {})
        self.ed_enabled = self.ed_config.get("enabled", False) # Off by default
        self.ed_t1 = self.ed_config.get("canny_threshold1", 100)
        self.ed_t2 = self.ed_config.get("canny_threshold2", 200)

        # --- NEW: Instantiate the classifier ---
        self.classifier = self._ThermalClassifier()
        # --- End of NEW ---

        print("[ThermalDetector] Multi-Strategy Detector Initialized.")
        if self.debug_mode:
            print("[ThermalDetector] DEBUG MODE: ON. A live feed window will be opened.")
            # Start the debug window thread
            asyncio.create_task(self._debug_window_thread())

    @staticmethod
    async def _debug_window_thread():
        """Creates and manages the debug window in a separate thread."""
        def window_loop():
            cv2.namedWindow("Thermal Debug", cv2.WINDOW_NORMAL)
            while True:
                # imshow is handled by the main detect thread
                # This thread just keeps the window open
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.destroyAllWindows()
        
        await asyncio.to_thread(window_loop)

    @staticmethod
    def _find_blobs_from_contours(contours: List[np.ndarray], 
                                  gray_frame: np.ndarray, 
                                  min_area: int) -> List[ThermalBlob]:
        """
        Converts raw contours into ThermalBlob objects.
        """
        blobs = []
        if not contours:
            return blobs
            
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > min_area:
                # Get bounding box for drawing and stats
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate the center of the contour
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue # Avoid division by zero
                    
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Calculate aspect ratio
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # --- Get thermal stats from the blob ---
                # Create a mask for the contour
                mask = np.zeros(gray_frame.shape, dtype="uint8")
                cv2.drawContours(mask, [contour], -1, 255, -1)
                
                # Get mean, min, max temp (grayscale value 0-255)
                # In a real system, you'd convert this from raw sensor data
                mean_val, max_val, min_val, _ = cv2.meanStdDev(gray_frame, mask=mask)
                
                # Simple approximation: 255 = 50C, 0 = 0C
                mean_temp = (mean_val[0][0] / 255.0) * 50.0
                max_temp = (max_val[0][0] / 255.0) * 50.0
                min_temp = (min_val[0][0] / 255.0) * 50.0
                
                blobs.append(ThermalBlob(
                    center_x=cX,
                    center_y=cY,
                    area=area,
                    mean_temp=mean_temp,
                    max_temp=max_temp,
                    min_temp=min_temp,
                    aspect_ratio=aspect_ratio,
                    bounding_box=(x,y,w,h)
                ))
            
        return blobs

    # --- Strategy 1: Simple Threshold ---
    def _process_simple_threshold(self, gray_frame: np.ndarray) -> List[ThermalBlob]:
        try:
            _value, thresh = cv2.threshold(gray_frame, self.st_threshold, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return self._find_blobs_from_contours(contours, gray_frame, self.min_area)
        except Exception as e:
            print(f"[ThermalDetector] SimpleThreshold failed: {e}")
            return []

    # --- Strategy 2: Adaptive (Statistical) Threshold ---
    def _process_adaptive_threshold(self, gray_frame: np.ndarray) -> List[ThermalBlob]:
        try:
            # Ensure block size is odd
            block_size = self.at_block_size if self.at_block_size % 2 != 0 else self.at_block_size + 1
                
            thresh = cv2.adaptiveThreshold(gray_frame, 255,
                                           cv2.ADAPTIVE_THRESH_MEAN_C,
                                           cv2.THRESH_BINARY, # Use BINARY, not INV, for white-hot
                                           block_size,
                                           -self.at_c_value) # Subtract C
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return self._find_blobs_from_contours(contours, gray_frame, self.min_area)
        except Exception as e:
            print(f"[ThermalDetector] AdaptiveThreshold failed: {e}")
            return []
    
    # --- Strategy 3: Edge Detection (Canny) ---
    def _process_edge_detection(self, gray_frame: np.ndarray) -> List[ThermalBlob]:
        try:
            # Blur to reduce noise before edge detection
            blurred = cv2.GaussianBlur(gray_frame, (5, 5), 0)
            edges = cv2.Canny(blurred, self.ed_t1, self.ed_t2)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # This is tricky, edge contours aren't filled. We'll find blobs inside.
            # For simplicity, we'll just treat the edge contours as blobs
            return self._find_blobs_from_contours(contours, gray_frame, self.min_area)
        except Exception as e:
            print(f"[ThermalDetector] EdgeDetection failed: {e}")
            return []

    # --- Main Detection Method ---
    async def detect(self) -> Optional[Detection]:
        """
        Gets a frame from the camera, finds blobs using all strategies,
        classifies them, and returns the best detection.
        """
        
        # 1. Get frame from the camera (e.g., numpy array)
        frame = await self.camera.get_frame()
        
        if frame is None:
            return None
            
        # 2. Prepare frame for processing (run in thread)
        def prep_frame(f):
            # Create a debug copy if needed
            debug_frame = f.copy() if self.debug_mode else None
            # Convert to grayscale
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            return gray, debug_frame
            
        try:
            gray_frame, debug_frame = await asyncio.to_thread(prep_frame, frame)
        except Exception as e:
            print(f"[ThermalDetector] CRITICAL: Frame prep failed: {e}")
            return None
            
        all_blobs: List[ThermalBlob] = []
        
        # 3. Run all enabled strategies
        tasks = []
        if self.st_enabled:
            tasks.append(asyncio.to_thread(self._process_simple_threshold, gray_frame))
        if self.at_enabled:
            tasks.append(asyncio.to_thread(self._process_adaptive_threshold, gray_frame))
        if self.ed_enabled:
            tasks.append(asyncio.to_thread(self._process_edge_detection, gray_frame))

        results: List[List[ThermalBlob]] = await asyncio.gather(*tasks)
        
        # 4. Process results
        blob_lists: Dict[str, List[ThermalBlob]] = {}
        res_index = 0
        if self.st_enabled:
            blob_lists["Simple"] = results[res_index]
            all_blobs.extend(results[res_index])
            res_index += 1
        if self.at_enabled:
            blob_lists["Adaptive"] = results[res_index]
            all_blobs.extend(results[res_index])
            res_index += 1
        if self.ed_enabled:
            blob_lists["Edge"] = results[res_index]
            all_blobs.extend(results[res_index])
            res_index += 1
            
        # TODO: Add blob merging logic here to combine overlapping blobs
        
        # 5. Classify all found blobs
        best_detection: Optional[Detection] = None
        
        for blob in all_blobs:
            target_type, confidence = self.classifier.classify_blob(blob)
            
            if target_type == TargetType.PERSON:
                print(f"[ThermalDetector] *** PERSON DETECTED! *** at ({blob.center_x}, {blob.center_y})")
                best_detection = Detection(
                    timestamp=time.monotonic(),
                    class_label=TargetType.PERSON.value,
                    confidence=confidence,
                    position=(blob.center_x, blob.center_y),
                    temperature=blob.mean_temp
                )
                if self.debug_mode:
                    x, y, w, h = blob.bounding_box
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), COLOR_YELLOW, 3)
                    cv2.putText(debug_frame, f"PERSON ({confidence:.2f})", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_YELLOW, 2)
                
                # Found a person, stop processing
                break
            
            # Draw other classified blobs in debug
            if self.debug_mode:
                x, y, w, h = blob.bounding_box
                if target_type == TargetType.BOAT:
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), COLOR_RED, 2)
                    cv2.putText(debug_frame, f"Boat ({confidence:.2f})", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RED, 1)
                elif target_type == TargetType.DEBRIS:
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), COLOR_BLUE, 1)
                    cv2.putText(debug_frame, f"Debris ({confidence:.2f})", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BLUE, 1)


        # 6. Show debug frame if enabled
        if self.debug_mode:
            await asyncio.to_thread(cv2.imshow, "Thermal Debug", debug_frame)

        # 7. Return the best detection (or None)
        return best_detection