from abc import ABC, abstractmethod
import numpy as np

class BaseEquation(ABC):
    def __init__(self, world_state, config):
        self.world = world_state
        self.config = config
        
    @abstractmethod
    def compute_gradient(self, drone_state): 
        pass

    # FIX: Added default debug implementation
    def compute_gradient_debug(self, drone_state):
        """
        Wrapper for compute_gradient that returns a default debug object.
        Ensures compatibility with Solver.solve().
        """
        gradient = self.compute_gradient(drone_state)
        
        # Import locally to avoid circular dependency issues
        from ..solver import SolverDebug
        
        debug = SolverDebug(
            gradient_travel=gradient,
            gradient_search=np.zeros(3),
            gradient_wind=np.zeros(3),
            cost_value=0.0
        )
        return gradient, debug