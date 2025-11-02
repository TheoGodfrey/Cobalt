"""
Kalman Filter-based target tracker for fusing detections over time.
"""
import time
import numpy as np
from typing import List, Tuple
from ..cameras.base import Detection
from ..position import Position

# A simple constant-velocity Kalman Filter placeholder.
# In a real system, you might use a library like 'filterpy'.
class KalmanTracker:
    """Manages the state of a single tracked target."""
    
    def __init__(self, initial_detection: Detection):
        # State: [x, y, vx, vy]
        self.state = np.array([
            initial_detection.position_image[0],
            initial_detection.position_image[1],
            0.0,
            0.0
        ])
        # Placeholder for state covariance
        self.covariance = np.eye(4) * 500
        
        self.track_id = int(time.time() * 1000) % 100000
        self.last_updated = time.time()
        self.age = 0
        self.hits = 1 # Number of frames this track has been updated
        self.source = initial_detection.source
        self.last_detection = initial_detection
    
    def predict(self, dt: float):
        """Predict the next state of the target."""
        # Simple constant velocity prediction
        # F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        # self.state = F @ self.state
        self.state[0] += self.state[2] * dt
        self.state[1] += self.state[3] * dt
        self.age += 1
        
    def update(self, detection: Detection):
        """Update the track with a new detection."""
        # This is a simplified update (measurement just replaces prediction)
        # A real Kalman update would fuse measurement and prediction.
        
        # Calculate velocity
        dt = time.time() - self.last_updated
        if dt > 0.01:
            self.state[2] = (detection.position_image[0] - self.state[0]) / dt
            self.state[3] = (detection.position_image[1] - self.state[1]) / dt
        
        # Update position
        self.state[0] = detection.position_image[0]
        self.state[1] = detection.position_image[1]
        
        self.last_updated = time.time()
        self.hits += 1
        self.age = 0 # Reset age since it was seen
        self.source = f"fused({self.source}, {detection.source})"
        self.last_detection = detection

    def get_pos(self) -> Tuple[float, float]:
        """Get the current estimated position."""
        return (self.state[0], self.state[1])
    
    def get_detection(self) -> Detection:
        """Get a Detection object representing the tracker's current state."""
        # Return the *last known* good detection, but update its
        # position and confidence based on the tracker's state.
        confidence = min(1.0, 0.7 + (self.hits * 0.05)) # Confidence grows with hits
        
        return Detection(
            position_image=(int(self.state[0]), int(self.state[1])),
            position_world=self.last_detection.position_world, # World pos is updated by behavior
            confidence=confidence,
            is_person=True,
            source='tracker',
            metadata={
                'track_id': self.track_id,
                'track_hits': self.hits,
                'track_age': self.age,
                'last_source': self.last_detection.source
            }
        )