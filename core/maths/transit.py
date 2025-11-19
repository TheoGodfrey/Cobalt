import numpy as np
from .base import BaseEquation

class RapidTransit(BaseEquation):
    def compute_gradient(self, drone_state):
        target = np.array(self.config['target_location'])
        direction = (target - drone_state.position_local)
        dist = np.linalg.norm(direction)
        if dist > 0: direction /= dist
        return direction * 100.0 # Sprint