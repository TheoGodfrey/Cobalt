import numpy as np
from dataclasses import dataclass

@dataclass
class SolverDebug:
    """
    Transparently explains the solver's decision.
    Sent to GCS for "Glass Box" visualization.
    """
    gradient_travel: np.ndarray  # "I want to go here to reach the target"
    gradient_search: np.ndarray  # "I want to go here to see more"
    gradient_wind: np.ndarray    # "The wind is pushing me here"
    cost_value: float          # "This is how 'bad' the current state is"

class ProbabilisticSolver:
    def __init__(self, config=None):
        config = config or {}
        self.equations = []
        self.last_debug = None
        
        # Kinematic Constraints
        self.max_speed_xy = config.get('max_speed_xy', 15.0) # m/s
        self.max_climb_rate = config.get('max_climb_rate', 5.0) # m/s
        self.max_descend_rate = config.get('max_descend_rate', 3.0) # m/s

    def set_equation(self, equation):
        """Sets the primary equation (Mission). Clears others."""
        self.equations = [equation]

    def add_equation(self, equation):
        """Adds a secondary equation (Constraint, Swarm)."""
        self.equations.append(equation)
        
    def clear_equations(self):
        self.equations = []

    def solve(self, drone_state):
        """
        Computes the net velocity vector [vx, vy, vz] (NED).
        """
        if not self.equations:
            return np.zeros(3)
        
        total_cmd = np.zeros(3)
        
        # Accumulate gradients
        # Note: Debug info from multiple equations is tough to merge cleanly.
        # We'll prioritize the PRIMARY (first) equation for the debug trace.
        primary_debug = None
        
        for i, eq in enumerate(self.equations):
            cmd, debug = eq.compute_gradient_debug(drone_state)
            total_cmd += cmd
            if i == 0:
                primary_debug = debug
        
        self.last_debug = primary_debug
        
        # Kinematic Saturation (Clamping)
        # 1. Horizontal Speed Limit
        v_xy = total_cmd[:2]
        speed_xy = np.linalg.norm(v_xy)
        if speed_xy > self.max_speed_xy:
            v_xy = (v_xy / speed_xy) * self.max_speed_xy
            total_cmd[:2] = v_xy
            
        # 2. Vertical Rate Limit (Z is positive down)
        # Climbing (Negative Z)
        if total_cmd[2] < -self.max_climb_rate:
            total_cmd[2] = -self.max_climb_rate
        # Descending (Positive Z)
        elif total_cmd[2] > self.max_descend_rate:
            total_cmd[2] = self.max_descend_rate
            
        return total_cmd

    def get_debug_info(self):
        return self.last_debug