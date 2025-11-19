import numpy as np
import math
import time

class LidarSensor:
    """
    Handles interaction with a LIDAR unit (1D Rangefinder or 3D Scanner).
    """
    def __init__(self, mode="mock", max_range=100.0):
        self.mode = mode
        self.max_range = max_range
        
        if self.mode == "mock":
            print("[Lidar] Initialized in MOCK mode.")
        else:
            # In real deployment, initialize Ouster/Livox/LightWare SDK here
            print("[Lidar] Initialized hardware connection.")

    def get_range_at_angle(self, azimuth_rad, elevation_rad):
        """
        Returns the range (meters) at a specific angle relative to drone body.
        Args:
            azimuth_rad: Horizontal angle (rad). 0 = Forward, + = Right.
            elevation_rad: Vertical angle (rad). 0 = Forward, + = Down.
        """
        if self.mode == "mock":
            # --- SIMULATION ---
            # Assume we are looking at the ocean surface at z=0
            # We assume the drone is at 20m altitude for this mock test
            altitude = 20.0 
            
            # Calculate total angle off nadir (straight down)
            # We assume elevation_rad input is relative to boresight (down)
            angle_off_nadir = math.sqrt(azimuth_rad**2 + elevation_rad**2)
            
            # Horizon check
            if math.cos(angle_off_nadir) < 0.01: 
                return self.max_range
            
            mock_dist = altitude / math.cos(angle_off_nadir)
            
            # Limit to max range
            if mock_dist > self.max_range:
                return self.max_range

            # Inject noise (+/- 5cm) and random "dropouts" (waves)
            if np.random.random() < 0.05: return 0.0 # Dropout
            
            # Return distance with Gaussian noise
            return mock_dist + np.random.normal(0, 0.05)

        # --- REAL HARDWARE ---
        # 1. Get latest point cloud or single ray
        # 2. Find point closest to (azimuth, elevation) ray
        # 3. Return that point's range
        return 0.0