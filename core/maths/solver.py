import numpy as np
from dataclasses import dataclass

@dataclass
class SolverDebug:
    """
    Transparently explains the solver's decision.
    Sent to GCS for "Glass Box" visualization.
    """
    gradient_travel: np.array  # "I want to go here to reach the target"
    gradient_search: np.array  # "I want to go here to see more"
    gradient_wind: np.array    # "The wind is pushing me here"
    cost_value: float          # "This is how 'bad' the current state is"

class ProbabilisticSolver:
    def __init__(self):
        self.active_equation = None
        self.last_debug = None

    def set_equation(self, equation):
        self.active_equation = equation

    def solve(self, drone_state):
        if not self.active_equation:
            return np.zeros(3)
        
        # The Equation now returns both the Command AND the Explanation
        cmd, debug = self.active_equation.compute_gradient_debug(drone_state)
        self.last_debug = debug
        return cmd

    def get_debug_info(self):
        return self.last_debug