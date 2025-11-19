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

    def update(self, limits):
        if limits: self.limits.update(limits)

    def filter_velocity(self, cmd, state):
        """
        Filters the velocity command to ensure 3D safety constraints.
        Args:
            cmd: [vx, vy, vz] Desired velocity (NED).
            state: DroneState object.
        """
        safe_cmd = cmd.copy()
        
        # NED: z is positive down. Altitude is -z.
        current_z = state.position_local[2]
        current_alt = -current_z
        
        # 1. Hard Deck Violation (Too Low)
        if current_alt < self.limits['min_altitude']:
            # We need to climb (Negative Z velocity)
            
            # If command is trying to descend (positive Z) or hold level (0), force climb.
            # If command is already climbing (negative Z), ensure it's climbing FAST ENOUGH.
            
            # Calculate required correction speed
            error = self.limits['min_altitude'] - current_alt
            
            # Proportional correction, capped at emergency rate
            # Must be negative to move UP in NED
            required_climb_vel = -min(error * 1.0, self.limits['emergency_climb_rate'])
            
            # Use the 'most upward' velocity (mathematical minimum in NED)
            # e.g. min(requested -1, required -2) = -2 (Faster climb)
            # e.g. min(requested +5, required -2) = -2 (Climb instead of dive)
            safe_cmd[2] = min(safe_cmd[2], required_climb_vel)
            
        # 2. Ceiling Violation (Too High)
        elif current_alt > self.limits['max_altitude']:
            # We need to descend (Positive Z velocity)
            if safe_cmd[2] < 0: safe_cmd[2] = 0 # Stop climbing
            safe_cmd[2] += 0.5 # Gentle nudge down
            
        # 3. Absolute Speed Limit
        speed = np.linalg.norm(safe_cmd)
        if speed > self.limits['max_speed']:
            safe_cmd = (safe_cmd / speed) * self.limits['max_speed']
        
        return safe_cmd