"""
World State - Enhanced with Hub Motion Tracking

NEW: Hub velocity and prediction for moving boat operations.
Critical for maritime RTH where hub is not stationary.
"""
import numpy as np
import time


class WorldState:
    """
    Unified world state for all perception models.
    
    Components:
    - Wind field (learned + instantaneous)
    - Probability field (target location belief)
    - Hub location and velocity (for RTH)
    - Fleet positions (for swarm coordination)
    """
    
    def __init__(self, config=None):
        config = config or {}
        
        # ============================================================
        # Hub (Home) Tracking - ENHANCED
        # ============================================================
        self.hub_location = np.array([0.0, 0.0, 0.0])  # [x, y, z] in local NED
        
        # NEW: Hub velocity for moving boat operations
        self.hub_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz] m/s
        self.hub_last_update = time.time()
        
        # Hub motion quality indicators
        self.hub_gps_quality = 0  # 0-9 (0=no fix, 9=RTK)
        self.hub_heading = 0.0    # radians (boat heading)
        
        # ============================================================
        # Fleet Coordination
        # ============================================================
        self.fleet_positions = {}  # {drone_id: np.array([x, y, z])}
        
        # ============================================================
        # Wind Model
        # ============================================================
        from core.state.wind import LearnedWindField
        self.wind = LearnedWindField(config.get('wind', {}))
        
        # ============================================================
        # Probability Model (Target Belief)
        # ============================================================
        from core.state.probability import ProbabilityModel
        prob_config = config.get('probability', {})
        self.probability = ProbabilityModel(
            resolution=prob_config.get('resolution', 10.0),
            bounds=prob_config.get('bounds', (-500, 500, -500, 500))
        )
        
        print("[WorldState] Initialized")
    
    # ============================================================
    # Hub Motion Management - NEW
    # ============================================================
    
    def update_hub(self, position, velocity=None, heading=None, gps_quality=None):
        """
        Update hub (boat) position and motion.
        
        Args:
            position: [x, y, z] in local NED frame
            velocity: [vx, vy, vz] in m/s (optional)
            heading: Boat heading in radians (optional)
            gps_quality: GPS fix quality 0-9 (optional)
        
        Called by:
        - MQTT listener when hub broadcasts its position
        - GCS when manually updating home position
        """
        self.hub_location = np.array(position, dtype=float)
        
        if velocity is not None:
            self.hub_velocity = np.array(velocity, dtype=float)
        
        if heading is not None:
            self.hub_heading = heading
        
        if gps_quality is not None:
            self.hub_gps_quality = gps_quality
        
        self.hub_last_update = time.time()
    
    def get_hub_with_prediction(self, lookahead_seconds=0.0):
        """
        Get hub position with motion prediction.
        
        Critical for RTH to moving boat:
        - If hub data is fresh (<1s old), use current position
        - If hub data is stale (>1s old), predict forward using velocity
        
        Args:
            lookahead_seconds: Additional lookahead time (for path planning)
        
        Returns:
            np.array([x, y, z]): Predicted hub position
        """
        # How old is our hub data?
        age = time.time() - self.hub_last_update
        
        # Warning if data is very stale
        if age > 10.0:
            print(f"[WorldState] WARNING: Hub data is {age:.1f}s old")
        
        # Total prediction time = data age + lookahead
        total_dt = age + lookahead_seconds
        
        # Predict forward using constant velocity model
        predicted_pos = self.hub_location + (self.hub_velocity * total_dt)
        
        return predicted_pos
    
    def get_hub_motion_quality(self):
        """
        Assess quality of hub motion tracking.
        
        Returns:
            str: "GOOD", "DEGRADED", or "LOST"
        """
        age = time.time() - self.hub_last_update
        
        if age < 2.0 and self.hub_gps_quality >= 5:
            return "GOOD"
        elif age < 10.0:
            return "DEGRADED"
        else:
            return "LOST"
    
    def is_hub_stationary(self, threshold=0.5):
        """
        Check if hub is stationary (ground-based operations).
        
        Args:
            threshold: Speed threshold in m/s
        
        Returns:
            bool: True if hub velocity < threshold
        """
        return np.linalg.norm(self.hub_velocity) < threshold
    
    # ============================================================
    # Fleet Management
    # ============================================================
    
    def get_nearest_neighbor(self, my_position):
        """
        Get nearest fleet member for swarm coordination.
        
        Args:
            my_position: np.array([x, y, z]) of current drone
        
        Returns:
            tuple: (drone_id, position, distance) or (None, None, inf)
        """
        if not self.fleet_positions:
            return None, None, float('inf')
        
        nearest_id = None
        nearest_pos = None
        nearest_dist = float('inf')
        
        for drone_id, pos in self.fleet_positions.items():
            dist = np.linalg.norm(pos - my_position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_id = drone_id
                nearest_pos = pos
        
        return nearest_id, nearest_pos, nearest_dist
    
    def get_fleet_centroid(self):
        """
        Get centroid of fleet (for coverage algorithms).
        
        Returns:
            np.array([x, y, z]) or None if no fleet data
        """
        if not self.fleet_positions:
            return None
        
        positions = list(self.fleet_positions.values())
        return np.mean(positions, axis=0)
    
    # ============================================================
    # Status Reporting
    # ============================================================
    
    def get_status(self):
        """
        Get world state summary for telemetry/debugging.
        
        Returns:
            dict: Summary of world state
        """
        return {
            'hub_location': self.hub_location.tolist(),
            'hub_velocity': self.hub_velocity.tolist(),
            'hub_velocity_mag': np.linalg.norm(self.hub_velocity),
            'hub_data_age': time.time() - self.hub_last_update,
            'hub_motion_quality': self.get_hub_motion_quality(),
            'hub_stationary': self.is_hub_stationary(),
            'fleet_count': len(self.fleet_positions),
            'wind_available': self.wind is not None,
            'probability_max': np.max(self.probability.field) if hasattr(self, 'probability') else 0
        }
    
    def print_status(self):
        """Print human-readable status"""
        status = self.get_status()
        
        print("\n" + "="*50)
        print("World State Status")
        print("="*50)
        print(f"Hub Location:     {status['hub_location']}")
        print(f"Hub Velocity:     {status['hub_velocity']} ({status['hub_velocity_mag']:.1f} m/s)")
        print(f"Hub Data Age:     {status['hub_data_age']:.1f}s")
        print(f"Hub Quality:      {status['hub_motion_quality']}")
        print(f"Hub Stationary:   {status['hub_stationary']}")
        print(f"Fleet Drones:     {status['fleet_count']}")
        print(f"Wind Model:       {'Available' if status['wind_available'] else 'Not Available'}")
        print(f"Target Max Prob:  {status['probability_max']:.3f}")
        print("="*50 + "\n")


# Example usage
if __name__ == "__main__":
    # Test hub motion tracking
    world = WorldState()
    
    print("Test 1: Stationary hub")
    world.update_hub(position=[100, 200, 0])
    predicted = world.get_hub_with_prediction(lookahead_seconds=5.0)
    print(f"Current: {world.hub_location}")
    print(f"Predicted (+5s): {predicted}")
    print(f"Motion quality: {world.get_hub_motion_quality()}")
    
    print("\nTest 2: Moving boat (5 m/s north)")
    world.update_hub(
        position=[100, 200, 0],
        velocity=[5.0, 0.0, 0.0],
        heading=0.0,
        gps_quality=8
    )
    
    # Simulate 3 second delay in data
    import time as pytime
    pytime.sleep(3)
    
    predicted = world.get_hub_with_prediction(lookahead_seconds=2.0)
    print(f"Current (stale): {world.hub_location}")
    print(f"Predicted (+3s stale +2s lookahead): {predicted}")
    print(f"Motion quality: {world.get_hub_motion_quality()}")
    
    # Status report
    world.print_status()