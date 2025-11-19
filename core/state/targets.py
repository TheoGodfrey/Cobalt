import numpy as np
import time
from dataclasses import dataclass, field

@dataclass
class TrackedObject:
    id: str
    label: str
    confidence: float
    position: np.array
    last_seen: float = field(default_factory=time.time)

class TargetRegistry:
    def __init__(self):
        self.targets = {}
        self.next_id = 1

    def update(self, pos, label, conf):
        pos = np.array(pos)
        best_id, best_dist = None, 20.0
        
        for tid, t in self.targets.items():
            dist = np.linalg.norm(t.position - pos)
            if dist < best_dist: best_id, best_dist = tid, dist
        
        if best_id:
            t = self.targets[best_id]
            t.position = 0.8 * pos + 0.2 * t.position # Smooth
            t.confidence = max(t.confidence, conf)
            t.last_seen = time.time()
            return best_id
        else:
            nid = f"T{self.next_id}"
            self.next_id += 1
            self.targets[nid] = TrackedObject(nid, label, conf, pos)
            return nid

    def get_best_target(self):
        if not self.targets: return None
        return max(self.targets.values(), key=lambda t: t.confidence)