import numpy as np
from .base import BaseEquation

class EnergyEfficientTransit(BaseEquation):
    def compute_gradient(self, drone_state):
        target = np.array(self.config['target_location'])
        pos = drone_state.position_local
        vec = target - pos
        dist = np.linalg.norm(vec)
        if dist < 1.0: return np.zeros(3)
        
        direction = vec / dist
        v_optimal = 12.0 
        wind = self.world.wind.get_at(pos)
        return (direction * v_optimal) + (wind * 0.5)