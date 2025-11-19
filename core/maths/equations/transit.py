import numpy as np
from .base import BaseEquation

class RapidTransit(BaseEquation):
    def compute_gradient(self, drone_state):
        """Rapid transit to target at maximum safe speed."""
        target = np.array(self.config['target_location'])
        direction = (target - drone_state.position_local)
        dist = np.linalg.norm(direction)
        
        if dist < 1.0:
            return np.zeros(3)
            
        if dist > 0:
            direction /= dist
            
        max_speed = self.config.get('max_speed', 15.0)
        return direction * max_speed