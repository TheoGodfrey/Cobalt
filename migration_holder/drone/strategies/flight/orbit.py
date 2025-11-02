"""
Orbit flight strategy - circle a target point
"""
import math
from core.position import Position
from core.config_models import OrbitConfig
from core.drone import Drone

class OrbitFlightStrategy:
    """Algorithm to fly in a circle around a target position."""
    
    def __init__(self, config: OrbitConfig):
        self.name = "orbit"
        self.description = "Fly in a circle around a target"
        self.config = config
        self.current_angle_deg = 0
    
    def get_next_position(self, drone: Drone, target_position: Position) -> Position:
        """
        Get the next waypoint in the orbit.
        Uses 8 discrete points for the circle.
        """
        
        # Increment angle, wrapping around at 360
        self.current_angle_deg = (self.current_angle_deg + 45) % 360
        angle_rad = math.radians(self.current_angle_deg)
        
        # Calculate new X, Y relative to the target
        x = target_position.x + (self.config.radius * math.cos(angle_rad))
        y = target_position.y + (self.config.radius * math.sin(angle_rad))
        
        # Z is relative to the target's Z (e.g., sea level)
        z = target_position.z + self.config.altitude_offset
        
        return Position(x=x, y=y, z=z)

# Factory function for composition
def create_orbit_flight_strategy(config: OrbitConfig):
    return OrbitFlightStrategy(config)
