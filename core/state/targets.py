import numpy as np
import time
from dataclasses import dataclass, field

@dataclass
class TrackedObject:
    id: str
    label: str
    confidence: float
    position: np.array # Now explicitly [x, y, z]
    velocity: np.array = field(default_factory=lambda: np.zeros(3)) # Estimated velocity
    last_seen: float = field(default_factory=time.time)

class TargetRegistry:
    def __init__(self):
        self.targets = {}
        self.next_id = 1

    def update(self, pos, label, conf):
        """
        Updates the registry with a new detection.
        
        Args:
            pos: [x, y, z] numpy array. If z is unknown (surface), pass z=0.
        """
        pos = np.array(pos)
        # Ensure 3D
        if pos.shape[0] == 2:
            pos = np.append(pos, 0.0) # Assume surface (z=0) if 2D provided
            
        best_id, best_dist = None, 20.0 # Association threshold (meters)
        
        for tid, t in self.targets.items():
            dist = np.linalg.norm(t.position - pos)
            if dist < best_dist: 
                best_id, best_dist = tid, dist
        
        if best_id:
            t = self.targets[best_id]
            # Simple Alpha-Beta filter for smoothing
            dt = time.time() - t.last_seen
            if dt > 0:
                # Estimate velocity (naive)
                new_vel = (pos - t.position) / dt
                t.velocity = 0.8 * t.velocity + 0.2 * new_vel
            
            t.position = 0.8 * pos + 0.2 * t.position # Smooth position
            t.confidence = max(t.confidence, conf)
            t.last_seen = time.time()
            return best_id
        else:
            nid = f"T{self.next_id}"
            self.next_id += 1
            self.targets[nid] = TrackedObject(
                id=nid, 
                label=label, 
                confidence=conf, 
                position=pos
            )
            return nid

    def get_best_target(self):
        if not self.targets: return None
        return max(self.targets.values(), key=lambda t: t.confidence)