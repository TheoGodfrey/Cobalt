import numpy as np

class SafetyMonitor:
    def __init__(self):
        self.limits = {"min_altitude": 5.0, "max_speed": 10.0}

    def update(self, limits):
        if limits: self.limits.update(limits)

    def filter(self, cmd, state):
        safe = cmd.copy()
        # 1. Hard Deck (Z is down, so < -min_alt is unsafe)
        if -state.pos[2] < self.limits['min_altitude']:
            if safe[2] > 0: safe[2] = 0 # Stop descent
            safe[2] -= 2.0 # Push up
        
        # 2. Speed Limit
        spd = np.linalg.norm(safe)
        if spd > self.limits['max_speed']:
            safe = (safe / spd) * self.limits['max_speed']
        
        return safe