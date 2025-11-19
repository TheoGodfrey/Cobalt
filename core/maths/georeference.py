import numpy as np
import math

class Georeferencer:
    def __init__(self, fov_deg_x=60.0, image_width=640, image_height=480):
        self.fov_x = math.radians(fov_deg_x)
        self.width = image_width
        self.height = image_height
        
        # Calculate focal length in pixels
        # tan(theta/2) = (w/2) / f
        self.f_px = (self.width / 2.0) / math.tan(self.fov_x / 2.0)

    def pixel_to_world(self, pixel_x, pixel_y, state, lidar_range=None):
        """
        Converts a 2D pixel to a 3D global coordinate using Sensor Fusion.
        
        Args:
            pixel_x, pixel_y: Target center on screen.
            state: DroneState object (containing position_local [x,y,z] and heading).
            lidar_range: (Optional) Direct distance to target in meters.
        
        Returns:
            np.array([x, y, z]) in NED frame, or None if solution impossible.
        """
        # 1. Calculate Ray in Camera Frame
        # Center is (0,0)
        cx = pixel_x - (self.width / 2.0)
        cy = pixel_y - (self.height / 2.0)
        
        # Angles relative to camera boresight
        # Assumption: Camera is pointing STRAIGHT DOWN (Nadir).
        # Body Frame: X=Forward, Y=Right, Z=Down
        
        # Mapping Image -> Body (Nadir Mount):
        # Image Up (-y)    = Body Forward (+x)
        # Image Right (+x) = Body Right (+y)
        
        # Ray slopes in Body Frame
        slope_fwd = -cy / self.f_px
        slope_right = cx / self.f_px
        
        heading = state.heading
        
        # STRATEGY A: LIDAR FUSION (High Precision)
        if lidar_range and lidar_range > 1.0 and lidar_range < 200.0:
            # We have the hypotenuse (r) and the directional slopes.
            # Vector in Body Frame (un-normalized) is [slope_fwd, slope_right, 1.0]
            # We need to normalize this vector and multiply by lidar_range
            
            v_body_un_norm = np.array([slope_fwd, slope_right, 1.0]) 
            norm = np.linalg.norm(v_body_un_norm)
            
            if norm < 1e-6: return None
            
            v_body = (v_body_un_norm / norm) * lidar_range
            
            # Rotate Body -> NED (using Heading only for now)
            # N = X cos(h) - Y sin(h)
            # E = X sin(h) + Y cos(h)
            # D = Z
            
            dn = (v_body[0] * math.cos(heading)) - (v_body[1] * math.sin(heading))
            de = (v_body[0] * math.sin(heading)) + (v_body[1] * math.cos(heading))
            dd = v_body[2]
            
            return state.position_local + np.array([dn, de, dd])

        # STRATEGY B: VISUAL RAYCAST (Flat Earth Fallback)
        else:
            drone_h = -state.position_local[2] # Altitude (Positive Up)
            if drone_h < 0.5: return None
            
            # Solve for Z intersection
            # scale * 1.0 = drone_h  => scale = drone_h
            # (Since Z component of our un-normalized vector is 1.0)
            
            body_x = slope_fwd * drone_h
            body_y = slope_right * drone_h
            
            # Rotate to NED
            dn = (body_x * math.cos(heading)) - (body_y * math.sin(heading))
            de = (body_x * math.sin(heading)) + (body_y * math.cos(heading))
            
            # Z is 0 (Surface)
            # state.z is -altitude. Target z is 0.
            # Result is [pos_n + dn, pos_e + de, 0]
            
            return np.array([state.position_local[0] + dn, 
                             state.position_local[1] + de, 
                             0.0])