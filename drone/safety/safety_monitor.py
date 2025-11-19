import numpy as np

class SafetyMonitor:
    def __init__(self):
        self.limits = {"min_altitude": 5.0, "max_speed": 10.0}

    def update(self, limits):
        if limits: self.limits.update(limits)

    def filter_velocity(self, cmd, state):
        """
        Filters the velocity command to ensure safety constraints (Hard Deck, Speed Limit).
        Renamed from 'filter' to match main.py usage.
        """
        safe = cmd.copy()
        
        # 1. Hard Deck (Z is down, so < -min_alt is unsafe)
        # state.position_local is [x, y, z]. Z is positive down.
        # If drone is at 10m altitude, Z is -10. -(-10) = 10 > 5 (Safe).
        current_alt = -state.position_local[2]
        
        if current_alt < self.limits['min_altitude']:
            # If we are below hard deck
            if safe[2] > 0: safe[2] = 0 # Stop descent (positive Z is down)
            safe[2] -= 2.0 # Active push up (negative Z is up)
        
        # 2. Speed Limit
        spd = np.linalg.norm(safe)
        if spd > self.limits['max_speed']:
            safe = (safe / spd) * self.limits['max_speed']
        
        return safe