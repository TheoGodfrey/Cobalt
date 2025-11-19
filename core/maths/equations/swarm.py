import numpy as np
from .base import BaseEquation
from ..solver import SolverDebug

class SwarmRepulsion(BaseEquation):
    def __init__(self, world_state, config):
        super().__init__(world_state, config)
        self.separation_dist = config.get('separation', 15.0) # meters
        self.gain = config.get('gain', 20.0)
        self.my_id = config.get('drone_id', 'unknown')

    def compute_gradient(self, drone_state):
        pos = drone_state.position_local
        repulsion = np.zeros(3)
        
        # Get list of neighbors from Comms/WorldState
        # Note: In MVP, this data might need to be injected into WorldState
        # We'll assume WorldState has a 'fleet_positions' dict
        neighbors = getattr(self.world, 'fleet_positions', {})
        
        count = 0
        for drone_id, n_pos in neighbors.items():
            if drone_id == self.my_id: continue
            
            # Vector from neighbor to me
            vec = pos - n_pos
            dist = np.linalg.norm(vec)
            
            # 3D Repulsion
            if dist < self.separation_dist:
                if dist > 0.1:
                    direction = vec / dist
                else:
                    direction = np.array([1.0, 0, 0]) # Random push
                
                # Linear falloff force
                force = self.gain * (self.separation_dist - dist) / self.separation_dist
                repulsion += direction * force
                count += 1
                
        return repulsion

    def compute_gradient_debug(self, drone_state):
        grad = self.compute_gradient(drone_state)
        return grad, SolverDebug(
            gradient_travel=np.zeros(3),
            gradient_search=grad, # Treat as search (coverage) pressure
            gradient_wind=np.zeros(3),
            cost_value=np.linalg.norm(grad)
        )