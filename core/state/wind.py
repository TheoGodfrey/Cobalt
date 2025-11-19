from abc import ABC, abstractmethod
import numpy as np
import time

class BaseWindModel(ABC):
    @abstractmethod
    def update(self, position, velocity): pass
    @abstractmethod
    def get_at(self, position): pass

class StaticWind(BaseWindModel):
    def __init__(self, config=None):
        config = config or {}
        self.vector = np.array(config.get('vector', [0.0, 0.0, 0.0]))
    
    def update(self, position, velocity): pass
    
    def get_at(self, position): return self.vector

class LearnedWindField(BaseWindModel):
    def __init__(self, config=None):
        config = config or {}
        self.obs = [] 
        self.sigma_s = config.get('sigma_s', 500.0) # Spatial correlation scale
        self.sigma_t = config.get('sigma_t', 300.0) # Temporal correlation scale
        
        # Vertical Profile Parameters (Log Law)
        self.z0 = config.get('roughness_length', 0.03) # Surface roughness (m)
        self.ref_h = config.get('reference_height', 10.0) # Height of measurements

    def update(self, pos, vel):
        """
        Updates the model with a new wind observation.
        Args:
            pos: [x, y, z] (z is altitude, positive up for wind logic)
            vel: [vx, vy, vz] wind vector
        """
        # Store observation with timestamp
        self.obs.append({'p': np.array(pos), 'v': np.array(vel), 't': time.time()})
        # Keep buffer manageable
        if len(self.obs) > 500: self.obs.pop(0)

    def get_at(self, pos):
        """
        Estimates wind vector at a query position (x, y, z).
        Applies a vertical logarithmic shear profile.
        """
        # Ensure pos is 3D
        if len(pos) == 2: pos = np.append(pos, 10.0) 
        
        query_alt = abs(pos[2]) # Assuming input might be NED (negative z) or Altitude
        if query_alt < 0.1: query_alt = 0.1 # Avoid log(0)

        # 1. Calculate Base Wind (at reference height) using Gaussian Kernel
        # We project all observations to reference height for the kernel
        # This simplifies the Gaussian regression to 2D+Time
        
        base_wind_sum = np.zeros(3)
        total_weight = 1e-6
        now = time.time()
        
        # If no data, return 0
        if not self.obs: return np.zeros(3)

        for o in self.obs:
            # Distance in 2D plane
            d2 = (pos[0] - o['p'][0])**2 + (pos[1] - o['p'][1])**2
            t2 = (now - o['t'])**2
            
            # Spatiotemporal Weight
            w = np.exp(-d2/(2*self.sigma_s**2)) * np.exp(-t2/(2*self.sigma_t**2))
            
            # Normalize observation to reference height before averaging
            # v_ref = v_obs * (ln(h_ref/z0) / ln(h_obs/z0))
            obs_alt = abs(o['p'][2])
            if obs_alt < 0.1: obs_alt = 0.1
            
            scaling_factor = np.log(self.ref_h / self.z0) / np.log(obs_alt / self.z0)
            v_ref_equiv = o['v'] * scaling_factor
            
            base_wind_sum += v_ref_equiv * w
            total_weight += w
            
        base_wind = base_wind_sum / total_weight
        
        # 2. Apply Vertical Profile to the estimated base wind
        # v(h) = v_ref * (ln(h/z0) / ln(h_ref/z0))
        shear_multiplier = np.log(query_alt / self.z0) / np.log(self.ref_h / self.z0)
        
        # Clamp multiplier to avoid unrealistic winds at extreme altitudes
        shear_multiplier = np.clip(shear_multiplier, 0.0, 3.0)
        
        return base_wind * shear_multiplier