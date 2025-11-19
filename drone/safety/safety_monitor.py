import numpy as np

class SafetyMonitor:
    def __init__(self, world_state, config=None):
        config = config or {}
        self.world = world_state # Store world state (needed for hub_location)
        self.limits = {
            "min_altitude": config.get("min_altitude", 5.0),
            "max_altitude": config.get("max_altitude", 120.0),
            "max_speed": config.get("max_speed", 15.0),
            "emergency_climb_rate": 2.0,
            
            # Battery Failsafe Config
            "min_flight_time_s": config.get("min_flight_time_s", 300.0), # 5 minutes
            "battery_failsafe_action": config.get("battery_failsafe_action", "RTL"), # "RTL" or "NONE"
            "total_flight_time_s": 1200.0,

            # --- NEW: Max Distance Failsafe Config ---
            "max_distance_m": config.get("max_distance_m", 500.0), # Max distance from hub (meters)
            "distance_failsafe_action": config.get("distance_failsafe_action", "RTL") # "RTL" or "NONE"
        }
        self.is_rtb_active = False

    def update(self, limits):
        if limits: 
            # Convert mission config minutes to seconds
            if "min_flight_time_minutes" in limits:
                limits['min_flight_time_s'] = limits['min_flight_time_minutes'] * 60.0
                del limits['min_flight_time_minutes']
            
            # Check for max_distance_m alias
            if "max_distance_from_hub" in limits:
                limits['max_distance_m'] = limits['max_distance_from_hub']
                del limits['max_distance_from_hub']
            
            self.limits.update(limits)

    def _estimate_remaining_time_s(self, battery_percent):
        """Simple linear estimation of flight time remaining."""
        return (battery_percent / 100.0) * self.limits["total_flight_time_s"]

    def _compute_rtb_command(self, pos, hub_pos):
        """Generates a high-priority velocity command to return to hub (boat)."""
        vec_to_hub = hub_pos - pos
        dist = np.linalg.norm(vec_to_hub)
        
        if dist < 5.0:
            self.is_rtb_active = False
            # Loiter over hub
            return np.array([0.0, 0.0, -1.0]) 
        
        self.is_rtb_active = True
        direction = vec_to_hub / dist
        
        rtb_speed = self.limits['max_speed'] * 0.9 
        
        # Override Z component to climb/descend towards a safe altitude (e.g., 10m)
        target_alt_z = -10.0 
        dz = target_alt_z - pos[2]
        cmd_z = np.clip(dz * 0.5, -self.limits['emergency_climb_rate'], self.limits['emergency_climb_rate'])

        # Calculate XY command
        cmd_xy = direction[:2] * rtb_speed
        
        # Ensure speed is within limits
        speed_xy = np.linalg.norm(cmd_xy)
        if speed_xy > rtb_speed:
            cmd_xy = (cmd_xy / speed_xy) * rtb_speed
            
        return np.array([cmd_xy[0], cmd_xy[1], cmd_z])

    def filter_velocity(self, cmd, state):
        """
        Filters the velocity command to ensure 3D safety constraints.
        """
        
        # 0. Failsafe Checks (Highest Priority Overrides)
        pos = state.position_local
        hub_pos = self.world.hub_location
        
        # --- NEW: Max Distance Failsafe Check ---
        if self.limits["distance_failsafe_action"] == "RTL":
            # Calculate 3D distance to hub
            distance = np.linalg.norm(pos - hub_pos)
            if distance > self.limits['max_distance_m']:
                print(f"[Safety] WARNING: Max distance exceeded ({distance:.1f}m > {self.limits['max_distance_m']:.1f}m). Initiating RTL.")
                return self._compute_rtb_command(pos, hub_pos)

        # 1. Battery Failsafe Check
        if self.limits["battery_failsafe_action"] == "RTL":
            time_left = self._estimate_remaining_time_s(state.battery)
            if time_left < self.limits['min_flight_time_s']:
                print(f"[Safety] WARNING: Low battery ({time_left:.1f}s left). Initiating RTL.")
                return self._compute_rtb_command(pos, hub_pos)

        safe_cmd = cmd.copy()
        
        # 2. Hard Deck/Ceiling Violation
        current_z = state.position_local[2]
        current_alt = -current_z
        
        # Hard Deck (Too Low)
        if current_alt < self.limits['min_altitude']:
            error = self.limits['min_altitude'] - current_alt
            required_climb_vel = -min(error * 1.0, self.limits['emergency_climb_rate'])
            safe_cmd[2] = min(safe_cmd[2], required_climb_vel)
            
        # Ceiling (Too High)
        elif current_alt > self.limits['max_altitude']:
            if safe_cmd[2] < 0: safe_cmd[2] = 0 
            safe_cmd[2] += 0.5 
            
        # 3. Absolute Speed Limit
        speed = np.linalg.norm(safe_cmd)
        if speed > self.limits['max_speed']:
            safe_cmd = (safe_cmd / speed) * self.limits['max_speed']
        
        return safe_cmd