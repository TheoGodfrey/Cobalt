"""
Sensor fusion detector - combines thermal + visual over time with a Kalman Tracker.
"""
import asyncio
import time
import numpy as np
from typing import List, Tuple, Optional
from ..cameras.base import Detection
from ..cameras.dual_camera import DualFrame
from .thermal_detector import ThermalDetector
from .visual_detector import VisualDetector
from .tracker import KalmanTracker # NEW
from ..config_models import DetectionConfig

class FusionDetector:
    """Fuses thermal and visual detections over time using a Kalman Tracker."""
    
    def __init__(self, config: DetectionConfig):
        """
        Initialize fusion detector
        """
        self.thermal_detector = ThermalDetector(config.thermal)
        self.visual_detector = VisualDetector(config.visual)
        
        self.config = config.fusion
        
        # NEW: List to hold active KalmanTracker instances
        self.tracks: List[KalmanTracker] = []
        self.last_update_time = time.time()
        
        # Tracker parameters
        self.max_age = 10 # Max frames to keep a track without a new detection
        self.min_hits_to_confirm = 3 # Min hits to be a "confirmed" target
        self.association_threshold = config.fusion.max_position_error # Pixels

    async def detect(self, dual_frame: DualFrame) -> List[Detection]:
        """
        Perform sensor fusion detection and tracking.
        
        Returns a list of stable, tracked detections.
        """
        # 1. Get new detections from both sensors concurrently
        thermal_detections, visual_detections = await asyncio.gather(
            self.thermal_detector.detect(dual_frame.thermal),
            self.visual_detector.detect(dual_frame.visual)
        )
        all_detections = thermal_detections + visual_detections
        
        # 2. Update the tracker with the new detections
        self._update_tracks(all_detections)
        
        # 3. Return confirmed tracks
        confirmed_detections = []
        for track in self.tracks:
            if track.hits >= self.min_hits_to_confirm:
                confirmed_detections.append(track.get_detection())
                
        return confirmed_detections

    def _update_tracks(self, detections: List[Detection]):
        """
        Update all active tracks with the new list of detections.
        This implements a simple association and tracking logic.
        """
        dt = time.time() - self.last_update_time
        self.last_update_time = time.time()
        
        # 1. Predict new state for all existing tracks
        for track in self.tracks:
            track.predict(dt)
            
        # 2. Associate new detections with existing tracks
        # (Using a simple greedy algorithm + distance threshold)
        matched_track_indices = set()
        matched_det_indices = set()
        
        for t_idx, track in enumerate(self.tracks):
            track_pos = track.get_pos()
            best_dist = float('inf')
            best_det_idx = -1
            
            for d_idx, det in enumerate(detections):
                if d_idx in matched_det_indices:
                    continue # This detection is already matched
                
                dist = np.linalg.norm(np.array(track_pos) - np.array(det.position_image))
                
                if dist < self.association_threshold and dist < best_dist:
                    best_dist = dist
                    best_det_idx = d_idx
            
            # If we found a match, update the track
            if best_det_idx != -1:
                track.update(detections[best_det_idx])
                matched_track_indices.add(t_idx)
                matched_det_indices.add(best_det_idx)

        # 3. Create new tracks for unmatched detections
        for d_idx, det in enumerate(detections):
            if d_idx not in matched_det_indices:
                # Only create new tracks for high-confidence detections
                if det.confidence > self.config.fusion_threshold:
                    new_track = KalmanTracker(det)
                    self.tracks.append(new_track)

        # 4. Prune stale tracks (unmatched and too old)
        self.tracks = [
            t for t_idx, t in enumerate(self.tracks)
            if t_idx in matched_track_indices or t.age < self.max_age
        ]