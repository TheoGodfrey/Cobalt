import numpy as np

# --- Constants from Thesis (Section 1.2) ---
# Tunable parameters for the specific airframe and sensor payload
H_REF = 10.0      # Reference altitude for sensor scaling (meters)
H_DECAY = 100.0   # Altitude where resolution decays significantly (meters)
K_DRAG = 2.0      # Aerodynamic drag penalty factor for crab angles
C_D = 0.05        # Drag coefficient (simplified)
BATTERY_MU = 5.0  # Soft barrier stiffness for energy urgency

def sensor_efficiency(h):
    """
    Calculates Sensor Efficiency eta(h).
    Formula: eta(h) = (h / (h + h_ref)) * exp(-h / h_decay)
    
    Physically:
    - Low h: Efficiency is low (poor coverage area).
    - High h: Efficiency is low (poor resolution).
    - Optimal h: The sweet spot in between.
    """
    if h <= 0: return 0.0
    term1 = h / (h + H_REF)
    term2 = np.exp(-h / H_DECAY)
    return term1 * term2

def sensor_efficiency_prime(h):
    """
    Calculates the derivative d(eta)/dh.
    Crucial for the gradient descent solver to know which way is 'up' 
    in terms of information gain.
    """
    if h <= 0: return 0.01 # Gradient push to take off
    
    # d/dh [ (h / (h+R)) * exp(-h/D) ]
    # Using product rule: u'v + uv'
    
    term_denom = h + H_REF
    exp_term = np.exp(-h / H_DECAY)
    
    # u = h / (h + R)  -> u' = R / (h + R)^2
    u = h / term_denom
    u_prime = H_REF / (term_denom**2)
    
    # v = exp(-h/D)    -> v' = -(1/D) * exp(-h/D)
    v = exp_term
    v_prime = -(1.0 / H_DECAY) * exp_term
    
    return (u_prime * v) + (u * v_prime)

def aerodynamic_ground_speed(v_air_mag, wind_vec, direction_vec):
    """
    Calculates estimated ground speed Phi given wind.
    
    Args:
        v_air_mag: Magnitude of drone's airspeed (e.g., 15 m/s).
        wind_vec: Wind vector [wx, wy, wz].
        direction_vec: Unit vector pointing to target [dx, dy, dz].
    
    Returns:
        Scalar estimated ground speed along the track.
    """
    # Decompose wind into parallel and perpendicular components relative to track
    w_parallel = np.dot(wind_vec, direction_vec)
    
    wind_perp_vec = wind_vec - (w_parallel * direction_vec)
    w_perp_sq = np.dot(wind_perp_vec, wind_perp_vec)
    
    # Ground Speed equation from Thesis Section 1.2
    # Phi = sqrt(V_air^2 - ||w_perp||^2) + w_parallel
    
    discriminant = v_air_mag**2 - w_perp_sq
    
    if discriminant < 0:
        # Wind is too strong to fly this heading directly (crab angle > 90)
        return 0.1 # Non-zero to prevent divide-by-zero, but very high cost
        
    return np.sqrt(discriminant) + w_parallel

def orientation_efficiency(velocity_vec, target_vec):
    """
    Calculates E_frame (orientation efficiency).
    Penalizes vectors that require high crab angles (high drag).
    """
    # Simplified: Dot product of body velocity vs desired track
    norm_v = np.linalg.norm(velocity_vec)
    norm_t = np.linalg.norm(target_vec)
    
    if norm_v < 0.1 or norm_t < 0.1: return 1.0
    
    cos_psi = np.dot(velocity_vec, target_vec) / (norm_v * norm_t)
    sin_sq_psi = 1.0 - cos_psi**2
    
    # Formula: 1 / (1 + k_drag * sin^2(crab))
    return 1.0 / (1.0 + K_DRAG * sin_sq_psi)

def energy_urgency(batt_current, batt_total, energy_req):
    """
    Calculates Barrier Function B(E_batt, E_req).
    """
    if batt_current <= 0: return 100.0 # Infinite cost
    
    ratio = energy_req / batt_current
    if ratio <= 1.0:
        return 1.0
    else:
        # Soft barrier that hardens as we exceed required energy
        return 1.0 + BATTERY_MU * (ratio - 1.0)**2