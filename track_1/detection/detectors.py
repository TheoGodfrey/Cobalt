# track1_shipping/detectors.py
#
# [MERGED FILE]
# Contains all detection-related classes:
# 1. Detection (Dataclass)
# 2. ClassicObjectDetector (from track_1/sensors/thermal_camera/detecter.py)
# 3. EdgeObjectDetector
# 4. ThermalObjectDetector (YOLO)
# 5. SalienceFilter
# 6. CentroidTracker

import numpy as np
import cv2  # For classic detectors
from ultralytics import YOLO  # For ML detector
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Dict, Any
from collections import OrderedDict

# --- 1. Detection Dataclass (Used by all) ---

@dataclass
class Detection:
    """Dataclass for a single detection from any detector."""
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    confidence: float                  # 0-1
    class_label: str                   # "person", "blob", "edge_blob"
    
    # Calculated fields for salience
    centroid: Tuple[int, int] = (0, 0)
    bbox_area: int = 0
    aspect_ratio: float = 1.0
    
    # Optional thermal data
    temperature: Optional[float] = None # (0-255 pixel value)


# --- 2. Classic Detector (from your detecter.py) ---

class ClassicObjectDetector:
    """
    [CLASSIC CV IMPLEMENTATION]
    Detects bright blobs using a classic CV pipeline.
    This is a lightweight, fast, non-ML alternative to YOLO.
    """
    def __init__(self):
        print("[Detector] ClassicObjectDetector (Threshold) initialized.")
        self.THRESHOLD_VALUE = 200 
        self.MIN_BLOB_AREA = 50     
        self.MAX_BLOB_AREA = 25000  
        
    def detect(self, thermal_frame: np.ndarray) -> List[Detection]:
        detections = []
        if thermal_frame is None or thermal_frame.size == 0:
            return detections
            
        blurred_frame = cv2.GaussianBlur(thermal_frame, (7, 7), 0)
        _, thresh_frame = cv2.threshold(
            blurred_frame, 
            self.THRESHOLD_VALUE, 
            255, 
            cv2.THRESH_BINARY
        )
        contours, _ = cv2.findContours(
            thresh_frame, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return detections

        for c in contours:
            area = cv2.contourArea(c)
            if not (self.MIN_BLOB_AREA < area < self.MAX_BLOB_AREA):
                continue 

            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0: continue
            
            aspect_ratio = w / h
            if not (0.3 < aspect_ratio < 3.0):
                continue
            
            cX = x + w // 2
            cY = y + h // 2
            
            mask = np.zeros(thermal_frame.shape, dtype="uint8")
            cv2.drawContours(mask, [c], -1, 255, -1) 
            mean_temp_val = cv2.mean(thermal_frame, mask=mask)[0]

            detections.append(
                Detection(
                    bbox=(x, y, x + w, y + h),
                    confidence=1.0,
                    class_label="blob",
                    centroid=(cX, cY),
                    bbox_area=area,
                    aspect_ratio=aspect_ratio,
                    temperature=mean_temp_val 
                )
            )
        return detections

# --- 3. Edge Detector ---

class EdgeObjectDetector:
    """
    [EDGE CV IMPLEMENTATION]
    Detects blobs by finding sharp thermal gradients (edges).
    """
    def __init__(self):
        print("[Detector] EdgeObjectDetector (Canny+Close) initialized.")
        self.CANNY_THRESHOLD_1 = 50
        self.CANNY_THRESHOLD_2 = 150
        self.MORPH_KERNEL_SIZE = (7, 7)
        self.MIN_BLOB_AREA = 50     
        self.MAX_BLOB_AREA = 25000  

    def detect(self, thermal_frame: np.ndarray) -> List[Detection]:
        detections = []
        if thermal_frame is None or thermal_frame.size == 0:
            return detections
            
        blurred_frame = cv2.GaussianBlur(thermal_frame, (7, 7), 0)
        canny_edges = cv2.Canny(
            blurred_frame, 
            self.CANNY_THRESHOLD_1, 
            self.CANNY_THRESHOLD_2
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, 
            self.MORPH_KERNEL_SIZE
        )
        closed_edges = cv2.morphologyEx(
            canny_edges, 
            cv2.MORPH_CLOSE, 
            kernel
        )
        contours, _ = cv2.findContours(
            closed_edges, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return detections

        for c in contours:
            area = cv2.contourArea(c)
            if not (self.MIN_BLOB_AREA < area < self.MAX_BLOB_AREA):
                continue

            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0: continue
            
            aspect_ratio = w / h
            if not (0.3 < aspect_ratio < 3.0):
                continue
            
            cX = x + w // 2
            cY = y + h // 2
            
            mask = np.zeros(thermal_frame.shape, dtype="uint8")
            cv2.drawContours(mask, [c], -1, 255, -1)
            mean_temp_val = cv2.mean(thermal_frame, mask=mask)[0]

            detections.append(
                Detection(
                    bbox=(x, y, x + w, y + h),
                    confidence=1.0,
                    class_label="edge_blob",
                    centroid=(cX, cY),
                    bbox_area=area,
                    aspect_ratio=aspect_ratio,
                    temperature=mean_temp_val
                )
            )
        return detections

# --- 4. YOLO Detector ---

class ThermalObjectDetector:
    """
    [REAL IMPLEMENTATION]
    Uses a pre-trained YOLOv8s model to detect objects.
    """
    def __init__(self, model_path="yolov8s.pt"):
        print(f"[Detector] Loading YOLO model from {model_path}...")
        self.model = YOLO(model_path)
        self.target_classes = [0] # 0 is 'person'
        print("[Detector] Model loaded.")

    def detect(self, thermal_frame: np.ndarray) -> List[Detection]:
        # Note: YOLO expects a 3-channel image.
        # We must convert our grayscale frame back to BGR.
        if len(thermal_frame.shape) == 2:
            frame_bgr = cv2.cvtColor(thermal_frame, cv2.COLOR_GRAY2BGR)
        else:
            frame_bgr = thermal_frame
            
        results = self.model(frame_bgr, classes=self.target_classes, verbose=False)

        detections = []
        if not results or len(results) == 0:
            return detections
            
        result = results[0] 
        
        for box in result.boxes:
            x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0]]
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            class_label = self.model.names[class_id] 

            w = x2 - x1
            h = y2 - y1
            if w == 0 or h == 0: continue
                
            centroid = (x1 + w // 2, y1 + h // 2)
            bbox_area = w * h
            aspect_ratio = w / h
            
            # Get temperature
            mask = np.zeros(thermal_frame.shape, dtype="uint8")
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            mean_temp_val = cv2.mean(thermal_frame, mask=mask)[0]

            detections.append(
                Detection(
                    bbox=(x1, y1, x2, y2),
                    confidence=confidence,
                    class_label=class_label,
                    centroid=centroid,
                    bbox_area=bbox_area,
                    aspect_ratio=aspect_ratio,
                    temperature=mean_temp_val
                )
            )
        return detections

# --- 5. Salience Filter ---

class SalienceFilter:
    """
    Filters detections using hardcoded rules (size, aspect ratio, temp).
    """
    def __init__(self, mission_type="mob"):
        self.mission_type = mission_type
        print(f"[Salience] Filter initialized for '{mission_type}' rules.")
        
        self.ruleset = {
            "mob_yolo": self._filter_yolo_detections,
            "mob_classic": self._filter_classic_detections,
            "mob_edge": self._filter_edge_detections,
        }
        
    def filter(self, detections: List[Detection], context: Dict[str, Any]) -> List[Tuple[Detection, float]]:
        if not detections:
            return []
            
        label = detections[0].class_label
        if label == "person":
            filter_fn = self.ruleset["mob_yolo"]
        elif label == "blob":
            filter_fn = self.ruleset["mob_classic"]
        elif label == "edge_blob":
            filter_fn = self.ruleset["mob_edge"]
        else:
            return [] # Unknown detection type
            
        scored_detections = []
        for det in detections:
            score = filter_fn(det, context)
            if score > 0:
                scored_detections.append((det, score))

        scored_detections.sort(key=lambda x: x[1], reverse=True)
        return scored_detections

    def _filter_yolo_detections(self, det: Detection, context: Dict[str, Any]) -> float:
        score = det.confidence
        # YOLO already filtered by size/shape, but we can double-check
        if not (50 < det.bbox_area < 25000): return 0
        if not (0.3 < det.aspect_ratio < 3.0): return 0
        return score + 0.2 # Salience bonus

    def _filter_classic_detections(self, det: Detection, context: Dict[str, Any]) -> float:
        # Classic detector already filtered size/shape. We check temp.
        # Tune this: 180-250 is a "bright" spot
        if 180 < det.temperature < 250:
            return 0.9 # High salience (right temp)
        else:
            return 0.1 # Low salience (wrong temp)

    def _filter_edge_detections(self, det: Detection, context: Dict[str, Any]) -> float:
        # Edge detector already filtered size/shape. We check temp.
        if 180 < det.temperature < 250:
            return 0.9 # High salience (right temp)
        else:
            return 0.1 # Low salience (wrong temp, e.g., foam)

# --- 6. Centroid Tracker ---

class CentroidTracker:
    """
    A simple object tracker based on centroid distance.
    This gives us "memory" to confirm a target over multiple frames.
    """
    def __init__(self, max_missed=25, max_misseg_frames=10): # Added max_misseg_frames from your file
        self.next_object_id = 0
        self.tracks = OrderedDict() # Stores {objectID: track_info}
        self.max_missed = max_misseg_frames
        self._lock = threading.Lock() # Make thread-safe

    def _register_track(self, detection: Detection):
        track_id = self.next_object_id
        self.tracks[track_id] = {
            "id": track_id,
            "centroid": detection.centroid,
            "hits": 1,
            "misses": 0,
            "last_detection": detection
        }
        self.next_object_id += 1

    def _deregister_track(self, object_id: int):
        del self.tracks[object_id]

    def update(self, salient_detections: List[Detection]) -> Dict:
        with self._lock:
            if len(salient_detections) == 0:
                for track_id in list(self.tracks.keys()):
                    self.tracks[track_id]["misses"] += 1
                    if self.tracks[track_id]["misses"] > self.max_missed:
                        self._deregister_track(track_id)
                return self.tracks

            new_centroids = np.array([d.centroid for d in salient_detections])
            
            track_ids = list(self.tracks.keys())
            if len(track_ids) > 0:
                track_centroids = np.array([t["centroid"] for t in self.tracks.values()])
                dist = np.linalg.norm(track_centroids[:, np.newaxis] - new_centroids, axis=2)
                used_detections_idx = set()
                
                for track_idx, track_id in enumerate(track_ids):
                    if dist.shape[1] == 0: break
                    
                    closest_det_idx = np.argmin(dist[track_idx])
                    min_dist = dist[track_idx, closest_det_idx]

                    if min_dist < 50: # 50 pixel matching threshold
                        detection = salient_detections[closest_det_idx]
                        self.tracks[track_id]["centroid"] = detection.centroid
                        self.tracks[track_id]["hits"] += 1
                        self.tracks[track_id]["misses"] = 0
                        self.tracks[track_id]["last_detection"] = detection
                        used_detections_idx.add(closest_det_idx)
                    else:
                        self.tracks[track_id]["misses"] += 1
                        if self.tracks[track_id]["misses"] > self.max_missed:
                            self._deregister_track(track_id)
                            
                for i in range(len(salient_detections)):
                    if i not in used_detections_idx:
                        self._register_track(salient_detections[i])
                        
            else:
                for d in salient_detections:
                    self._register_track(d)
                    
            return self.tracks