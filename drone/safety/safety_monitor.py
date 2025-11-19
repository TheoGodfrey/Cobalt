# drone/safety/safety_monitor.py (Production Version)
import numpy as np
from core.types import DroneState

class SafetyMonitor:
    def __init__(self, world_state, config=None):
        self.world = world_state
        self.config = config or {}
        
        # Limits
        self.min_alt = self.config.get('min_altitude', 5.0)
        self.max_alt = self.config.get('max_altitude', 120.0)
        self.max_dist = self.config.get('max_distance_m', 500.0)
        self.max_speed = self.config.get('max_speed', 15.0)
        self.min_batt = self.config.get('min_battery_level', 20.0) # %
        
        # State
        self.home_pos = np.zeros(3) 

    def update_home(self, pos_array):
        self.home_pos = np.array(pos_array)

    def filter_velocity(self, cmd, state: DroneState):
        pos = state.position_local
        
        # 1. CRITICAL: Battery Failsafe
        if state.battery < self.min_batt:
            print(f"[Safety] Low Battery ({state.battery:.1f}%). RTB.")
            return self._get_rtb_vector(pos)

        # 2. CRITICAL: Max Distance Failsafe (Dynamic Home)
        dist_to_home = np.linalg.norm(pos - self.home_pos)
        if dist_to_home > self.max_dist:
            print(f"[Safety] Max Range Exceeded ({dist_to_home:.0f}m). RTB.")
            return self._get_rtb_vector(pos)

        # 3. Hard Deck (Floor)
        # Note: in NED, altitude is -z. So -pos[2] is height.
        if -pos[2] < self.min_alt:
            # If too low, override Z velocity to climb
            cmd[2] = min(cmd[2], -1.0) 

        # 4. Ceiling
        elif -pos[2] > self.max_alt:
            # If too high, override Z velocity to descend
            cmd[2] = max(cmd[2], 1.0)

        # 5. Speed Limit (Clamp magnitude)
        speed = np.linalg.norm(cmd)
        if speed > self.max_speed:
            cmd = (cmd / speed) * self.max_speed

        return cmd

    def _get_rtb_vector(self, current_pos):
        """Calculates a safe return vector."""
        vec = self.home_pos - current_pos
        dist = np.linalg.norm(vec)
        
        if dist < 1.0: 
            return np.array([0.0, 0.0, 0.5]) # Land slowly if home
            
        # Normalize and set speed
        velocity = (vec / dist) * (self.max_speed * 0.8)
        
        # Simple 'Safe Altitude' logic:
        # If we are below 20m, climb while returning.
        if -current_pos[2] < 20.0:
             velocity[2] = -2.0 # Climb at 2 m/s
             
        return velocity