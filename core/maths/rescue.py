import numpy as np
from .base import BaseEquation
from solver import SolverDebug

class UnifiedRescue(BaseEquation):
    def compute_gradient_debug(self, drone_state):
        pos = drone_state.position_local
        grad_travel = np.zeros(3)
        grad_search = np.zeros(3)
        
        # 1. Travel Term
        targets = self.world.probability.get_active_cells(threshold=0.001)
        for tx, ty, prob in targets:
            dx, dy = tx - pos[0], ty - pos[1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist > 1.0:
                pull = prob / (dist + 1.0)
                grad_travel[0] += pull * (dx/dist)
                grad_travel[1] += pull * (dy/dist)

        # 2. Search Term
        h = -pos[2]
        optimal_h = 50.0 if len(targets) > 20 else 10.0
        grad_search[2] = -(optimal_h - h) * 0.5 
        
        # Total
        total_cmd = (grad_travel + grad_search) * 5.0
        
        # Transparent Debug Object
        debug = SolverDebug(
            gradient_travel=grad_travel,
            gradient_search=grad_search,
            gradient_wind=self.world.wind.get_at(pos),
            cost_value=0.0 # Placeholder
        )
        
        return total_cmd, debug