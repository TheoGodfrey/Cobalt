import numpy as np

class SafetyMonitor:
    def __init__(self, config=None):
        config = config or {}
        self.limits = {
            "min_altitude": config.get("min_altitude", 5.0),   # Hard Deck (m)
            "max_altitude": config.get("max_altitude", 120.0), # Ceiling (m)
            "max_speed": config.get("max_speed", 15.0),        # Absolute max speed
            "emergency_climb_rate": 2.0
        }
        # State tracking for hysteresis
        self.is_below_deck = False

    def update(self, limits):
        if limits: self.limits.update(limits)

    def filter_velocity(self, cmd, state):
        """
        Filters the velocity command to ensure 3D safety constraints.
        
        Args:
            cmd: [vx, vy, vz] Desired velocity (NED).
            state: DroneState object.
            
        Returns:
            [vx, vy, vz] Safe velocity command.
        """
        safe_cmd = cmd.copy()
        
        # 1. Altitude Check (NED: z is positive down, altitude = -z)
        # state.position_local is [x, y, z]
        current_z = state.position_local[2]
        current_alt = -current_z
        
        # Hard Deck Violation (Too Low)
        if current_alt < self.limits['min_altitude']:
            # Override Z command to climb
            # Negative Z velocity = Climb
            if safe_cmd[2] > 0: safe_cmd[2] = 0 # Stop descending
            
            # Apply active correction (Proportional to violation)
            error = self.limits['min_altitude'] - current_alt
            correction = -min(error * 1.0, self.limits['emergency_climb_rate'])
            
            # Ensure we are climbing at least a little bit
            safe_cmd[2] = min(safe_cmd[2], correction) 
            
        # Ceiling Violation (Too High)
        elif current_alt > self.limits['max_altitude']:
            # Override Z command to descend
            if safe_cmd[2] < 0: safe_cmd[2] = 0 # Stop climbing
            
            safe_cmd[2] += 0.5 # Gentle nudge down
            
        # 2. Absolute Speed Limit
        speed = np.linalg.norm(safe_cmd)
        if speed > self.limits['max_speed']:
            safe_cmd = (safe_cmd / speed) * self.limits['max_speed']
        
        return safe_cmd