import numpy as np
from .base import BaseEquation

class EnergyEfficientTransit(BaseEquation):
    def compute_gradient(self, drone_state):
        """Energy-efficient transit to a target location."""
        target = np.array(self.config['target_location'])
        pos = drone_state.position_local
        vec = target - pos
        dist = np.linalg.norm(vec)
        
        if dist < 1.0:
            return np.zeros(3)
        
        direction = vec / dist
        max_speed = self.config.get('max_speed', 15.0)
        optimal_speed = max_speed * 0.8  # 80% for energy efficiency
        
        wind = self.world.wind.get_at(pos)
        
        # Command includes wind compensation
        return (direction * optimal_speed) + (wind * 0.3)