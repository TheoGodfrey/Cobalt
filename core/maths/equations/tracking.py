import numpy as np
import time
import math
from .base import BaseEquation
from ..solver import SolverDebug

class DynamicTracking(BaseEquation):
    def __init__(self, world_state, config):
        super().__init__(world_state, config)
        
        # Targeting Config
        self.target_id = config.get('target_id', None)
        
        # Standoff Config (Where to hover relative to target)
        # Default: Hover directly above
        self.standoff_dist = config.get('standoff_dist', 0.0) # 2D distance
        self.standoff_alt = config.get('standoff_alt', 20.0)  # Height above target
        
        # Control Gains
        self.kp = config.get('gain_p', 2.0) # Position Error Gain
        self.kv = config.get('gain_v', 1.0) # Velocity Feedforward Gain

        # --- Parallax Oscillation Config ---
        self.parallax_axis = config.get('parallax_axis', 'z') # 'z' (altitude) or 'xy' (orbit)
        self.osc_amplitude = config.get('oscillation_amplitude', 0.0) # Meters
        self.osc_period = config.get('oscillation_period', 10.0)      # Seconds
        
        self.start_time = time.time()

    def _get_oscillation_offset(self):
        """
        Calculates the position (delta) and velocity (vel) of the oscillation.
        """
        if self.osc_amplitude <= 0.01:
            return np.zeros(3), np.zeros(3)

        t = time.time() - self.start_time
        omega = (2 * math.pi) / self.osc_period
        
        # Position: x(t) = A * sin(omega * t)
        sine_val = math.sin(omega * t)
        # Velocity: v(t) = A * omega * cos(omega * t)
        cos_val = math.cos(omega * t)
        
        pos_offset = np.zeros(3)
        vel_offset = np.zeros(3)

        if self.parallax_axis == 'z':
            # Bob up and down
            pos_offset[2] = self.osc_amplitude * sine_val
            vel_offset[2] = self.osc_amplitude * omega * cos_val
            
        elif self.parallax_axis == 'y':
            # Sway side-to-side
            pos_offset[1] = self.osc_amplitude * sine_val
            vel_offset[1] = self.osc_amplitude * omega * cos_val
            
        return pos_offset, vel_offset

    def compute_gradient(self, drone_state):
        # 1. Acquire Target
        target = None
        if self.target_id:
            target = self.world.targets.targets.get(self.target_id)
        else:
            target = self.world.targets.get_best_target()

        if not target:
            return np.zeros(3)

        # 2. Calculate Nominal Setpoint (Static Hover)
        tgt_pos = target.position
        
        # Define the "Ideal" position relative to target (Negative Z is Up)
        desired_pos = np.array([tgt_pos[0], tgt_pos[1], tgt_pos[2] - self.standoff_alt])
        
        # 3. Add Parallax Oscillation
        osc_pos, osc_vel = self._get_oscillation_offset()
        
        # The final dynamic setpoint
        target_point = desired_pos + osc_pos
        
        # 4. Compute Error
        # Vector from Drone -> Setpoint
        error_vec = target_point - drone_state.position_local
        dist = np.linalg.norm(error_vec)
        
        # Direction
        if dist > 0.01:
            u_error = error_vec / dist
        else:
            u_error = np.zeros(3)
            
        # 5. Control Law: P-Controller + Feedforward
        # Gradient = (Kp * Error) + (Target_Vel) + (Oscillation_Vel)
        
        # Target velocity (from Kalman Filter or raw estimate)
        tgt_vel = getattr(target, 'velocity', np.zeros(3))
        
        # Note: Z is positive down (NED), but our oscillation math handles signs consistently
        cmd_vel = (error_vec * self.kp) + (tgt_vel * self.kv) + (osc_vel * self.kv)
        
        # Clamp for sanity
        speed = np.linalg.norm(cmd_vel)
        if speed > 25.0:
            cmd_vel = (cmd_vel / speed) * 25.0
            
        return cmd_vel

    def compute_gradient_debug(self, drone_state):
        grad = self.compute_gradient(drone_state)
        return grad, SolverDebug(
            gradient_travel=grad,
            gradient_search=np.zeros(3),
            gradient_wind=np.zeros(3),
            cost_value=np.linalg.norm(grad)
        )