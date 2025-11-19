import numpy as np
import time
from .base import BaseEquation
from ..solver import SolverDebug

# Import constitutive libraries
from maths.constitutative import (
    sensor_efficiency, 
    sensor_efficiency_prime, 
    aerodynamic_ground_speed,
    energy_urgency
)
from ..risk import EntropicRisk

class UnifiedRescue(BaseEquation):
    def __init__(self, world_state, config):
        super().__init__(world_state, config)
        
        # Risk & Physics Config
        theta = config.get('risk_theta', 0.1)
        self.risk_model = EntropicRisk(theta=theta)
        self.airspeed = config.get('cruise_speed', 15.0)
        
        # Phase Transition Config
        self.start_time = time.time()
        self.beta = config.get('decay_beta', 0.005) # 1/beta = approx duration of search phase
        
        # Optimization Config
        self.monte_carlo_samples = config.get('mc_samples', 200) # Max voxels to sample per cycle

    def compute_gradient_debug(self, drone_state):
        """
        Calculates the negative gradient of the Unified Potential J with 
        Time-Decay and Monte Carlo estimation.
        """
        
        # 1. State Extraction & Setup
        pos_n, pos_e, pos_d = drone_state.position_local
        altitude = -pos_d 
        wind_vec = self.world.wind.get_at(drone_state.position_local)
        
        # 2. Calculate Phase Weight (Time Decay)
        # w(t) = exp(-beta * t)
        mission_time = time.time() - self.start_time
        phase_weight = np.exp(-self.beta * mission_time)
        
        # 3. Get Active Voxels (The Belief)
        # Threshold filters out empty space
        all_targets = self.world.probability.get_active_cells(threshold=0.0001)
        
        # --- MONTE CARLO SAMPLING ---
        # If we have too many voxels, calculating the full integral is slow.
        # We sample N voxels based on their probability mass distribution.
        num_targets = len(all_targets)
        
        if num_targets == 0:
             return np.array([0.0, 0.0, -2.0]), SolverDebug(np.zeros(3), np.zeros(3), wind_vec, 0)

        if num_targets > self.monte_carlo_samples:
            # Extract probabilities to use as weights
            # all_targets structure: [(x, y, z, p), ...]
            probs = np.array([t[3] for t in all_targets])
            probs /= probs.sum() # Normalize to sum to 1
            
            # Random choice indices
            indices = np.random.choice(num_targets, self.monte_carlo_samples, p=probs)
            targets = [all_targets[i] for i in indices]
            
            # Reweight factor (Total Mass / N_Samples) to keep magnitude consistent
            # Not strictly necessary for gradient direction, but good for debugging cost
            sample_weight_factor = (1.0 / self.monte_carlo_samples) 
        else:
            targets = all_targets
            sample_weight_factor = 1.0

        # 4. Compute Gradients
        
        # --- A. Mission Gradient (The Search) ---
        eta = sensor_efficiency(altitude)
        eta_prime = sensor_efficiency_prime(altitude)
        
        grad_mission_x = 0.0
        grad_mission_y = 0.0
        grad_mission_h = 0.0
        
        # Centroid accumulators for Standby term
        centroid_x, centroid_y, centroid_z, mass_sum = 0.0, 0.0, 0.0, 0.0

        for tx, ty, tz, prob in targets:
            # Accumulate Centroid (using sampled prob)
            centroid_x += tx * prob
            centroid_y += ty * prob
            centroid_z += tz * prob
            mass_sum += prob

            # --- Gradient Logic (Same as Phase 2, simplified for brevity) ---
            dx, dy, dh = tx - pos_n, ty - pos_e, tz - altitude 
            dist = np.sqrt(dx**2 + dy**2 + dh**2)
            if dist < 1.0: dist = 1.0
            
            u_x, u_y, u_h = dx/dist, dy/dist, dh/dist
            
            # Cost C
            speed_est = aerodynamic_ground_speed(self.airspeed, wind_vec, np.array([u_x, u_y, -u_h]))
            speed_est = max(0.1, speed_est)
            time_to_go = dist / speed_est
            risk_cost = self.risk_model.compute_cost(time_to_go, 0.1 * time_to_go)
            
            # Gradients
            term1_h = eta_prime * prob * risk_cost * sample_weight_factor
            w = eta * prob * sample_weight_factor
            
            grad_mission_x += w * u_x
            grad_mission_y += w * u_y
            grad_mission_h += (w * u_h) - term1_h

        # --- B. Standby Gradient (The Intercept) ---
        # Pulls drone to the centroid of probability mass
        grad_standby_x = 0.0
        grad_standby_y = 0.0
        grad_standby_h = 0.0
        
        if mass_sum > 0:
            cx, cy, cz = centroid_x/mass_sum, centroid_y/mass_sum, centroid_z/mass_sum
            
            # Simple harmonic pull to centroid
            # We ignore wind here, it's just a geometric attractor
            dx, dy, dh = cx - pos_n, cy - pos_e, cz - altitude
            dist_c = np.sqrt(dx**2 + dy**2 + dh**2)
            
            if dist_c > 1.0:
                grad_standby_x = dx / dist_c
                grad_standby_y = dy / dist_c
                grad_standby_h = dh / dist_c

        # 5. Fusion (Time Decay)
        # Total Gradient = Phase_Weight * Mission + (1 - Phase_Weight) * Standby
        
        total_x = phase_weight * grad_mission_x + (1.0 - phase_weight) * grad_standby_x * 0.5 # scale standby
        total_y = phase_weight * grad_mission_y + (1.0 - phase_weight) * grad_standby_y * 0.5
        
        # Altitude needs careful handling
        # Mission pushes to h_opt. Standby pushes to target z (likely 0).
        # As phase_weight drops, drone naturally descends to intercept.
        total_h = phase_weight * grad_mission_h + (1.0 - phase_weight) * grad_standby_h * 0.5

        # Convert h (up) to z (down)
        cmd_x = total_x * 100.0
        cmd_y = total_y * 100.0
        cmd_z = -total_h * 50.0
        
        debug = SolverDebug(
            gradient_travel=np.array([cmd_x, cmd_y, 0]),
            gradient_search=np.array([0, 0, cmd_z]),
            gradient_wind=wind_vec,
            cost_value=phase_weight
        )
        
        return np.array([cmd_x, cmd_y, cmd_z]), debug