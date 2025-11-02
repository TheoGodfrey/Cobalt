"""
Navigation, Geolocation, and Coordinate Transformation Utilities.
"""
import numpy as np
import math
from typing import Tuple
from .position import Position
from .drone import Telemetry
from .config_models import CameraIntrinsics # <-- FIX: Was CameraIntrinsicsConfig

class CameraIntrinsicsHelper: # <-- FIX: Renamed class to avoid conflict
    """A helper class to hold camera intrinsic parameters."""
    def __init__(self, config: CameraIntrinsics): # <-- FIX: Was CameraIntrinsicsConfig
        self.fx = config.focal_length_x
        self.fy = config.focal_length_y
        self.cx = config.principal_point_x
        self.cy = config.principal_point_y
        self.width = config.width
        self.height = config.height
        
        # Pre-compute the inverse intrinsics matrix
        self.K_inv = np.array([
            [1/self.fx, 0, -self.cx/self.fx],
            [0, 1/self.fy, -self.cy/self.fy],
            [0, 0, 1]
        ])

class Attitude:
    """A helper class to hold attitude in radians."""
    def __init__(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)

def _get_rotation_matrix(attitude: Attitude) -> np.ndarray:
    """Calculates the 3D rotation matrix from drone to world frame."""
    # Z-Y-X (Yaw-Pitch-Roll) rotation
    cos_r = math.cos(attitude.roll_rad)
    sin_r = math.sin(attitude.roll_rad)
    cos_p = math.cos(attitude.pitch_rad)
    sin_p = math.sin(attitude.pitch_rad)
    cos_y = math.cos(attitude.yaw_rad)
    sin_y = math.sin(attitude.yaw_rad)

    # Rotation matrix for roll (around X)
    R_x = np.array([
        [1, 0, 0],
        [0, cos_r, -sin_r],
        [0, sin_r, cos_r]
    ])

    # Rotation matrix for pitch (around Y)
    R_y = np.array([
        [cos_p, 0, sin_p],
        [0, 1, 0],
        [-sin_p, 0, cos_p]
    ])

    # Rotation matrix for yaw (around Z)
    R_z = np.array([
        [cos_y, -sin_y, 0],
        [sin_y, cos_y, 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: ZYX
    # This transforms from body (drone) frame to world (NED/ENU) frame
    R = R_z @ R_y @ R_x
    return R

def image_to_world_position(pixel: Tuple[int, int],
                            drone_telemetry: Telemetry,
                            intrinsics: CameraIntrinsicsHelper, # <-- FIX
                            ground_level_z: float = 0.0) -> Position:
    """
    Performs geolocation (ray-casting) to find the 3D world position
    of a pixel, assuming a flat ground plane.
    
    Args:
        pixel: (x, y) pixel coordinates from the image.
        drone_telemetry: The full Telemetry object at the time of capture.
        intrinsics: The CameraIntrinsicsHelper object for the camera used.
        ground_level_z: The Z-coordinate of the ground (e.g., 0.0 for sea level).
        
    Returns:
        A Position object with the estimated (x, y, z) world coordinates.
    """
    
    # 1. Pixel to Normalized Camera Coordinates
    # Create a homogenous pixel vector: [u, v, 1]
    pixel_vec = np.array([pixel[0], pixel[1], 1.0])
    
    # Un-project pixel to a 3D vector in the camera's coordinate frame
    # v_cam = K_inv @ pixel_vec
    # This vector (x, y, 1) points from the camera center *through* the pixel.
    # Assumes camera X is right, Y is down, Z is forward.
    v_cam = intrinsics.K_inv @ pixel_vec
    
    # 2. Get Drone Attitude
    attitude = Attitude(
        drone_telemetry.attitude_roll,
        drone_telemetry.attitude_pitch,
        drone_telemetry.attitude_yaw
    )
    
    # 3. Get Rotation Matrix (Drone Body to World)
    # TODO: This assumes camera frame == drone body frame.
    # In reality, you'd have another R_cam_to_body transform.
    R_body_to_world = _get_rotation_matrix(attitude)
    
    # 4. Transform Camera Vector to World Frame
    # v_world = R_body_to_world @ v_cam
    v_world = R_body_to_world @ v_cam
    
    # 5. Ray-Plane Intersection
    # Ray origin (drone's current position)
    P0 = np.array([
        drone_telemetry.position.x,
        drone_telemetry.position.y,
        drone_telemetry.position.z
    ])
    
    # Ray direction (the rotated vector)
    V = v_world
    
    # Plane definition (a flat plane at ground_level_z)
    P_normal = np.array([0.0, 0.0, 1.0]) # Normal vector pointing up
    P_origin = np.array([0.0, 0.0, ground_level_z]) # A point on the plane

    # Check if ray is parallel to the plane (e.g., drone looking at horizon)
    V_dot_n = V @ P_normal
    if abs(V_dot_n) < 1e-6:
        # Ray is parallel or pointing away, cannot intersect
        return drone_telemetry.position # Return drone position as fallback
        
    # Calculate intersection parameter 't'
    t = ((P_origin - P0) @ P_normal) / V_dot_n
    
    if t < 0:
        # Intersection is *behind* the camera (e.g., drone is below ground)
        return drone_telemetry.position
    
    # 6. Calculate Intersection Point
    P_intersect = P0 + t * V
    
    return Position(x=P_intersect[0], y=P_intersect[1], z=P_intersect[2])