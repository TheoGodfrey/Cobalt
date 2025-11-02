"""
Lawnmower search algorithm - for systematic patrol
"""
from core.position import Position
from core.config_models import LawnmowerConfig
from core.drone import Drone

class LawnmowerSearchStrategy:
    """Systematic 'lawnmower' grid search."""
    
    def __init__(self, config: LawnmowerConfig):
        self.name = "lawnmower"
        self.description = "Systematic grid search pattern"
        self.config = config
        self.current_leg = 0
        
    def get_next_position(self, drone: Drone, search_area: Position, search_size: float) -> Position | None:
        """
        Get the next waypoint in the lawnmower pattern.
        Returns None if the pattern is complete.
        """
        if self.current_leg > self.config.num_legs:
            return None # Signal pattern is complete

        # Calculate leg start and end
        # Assumes search_area is the center (0,0) and size is total width
        half_width = search_size / 2.0
        x_start = -half_width
        
        # Calculate Y position for this leg
        # Start at one edge and move across
        y_pos = -half_width + (self.current_leg * self.config.spacing)
        
        # Check if we've gone past the boundary
        if y_pos > half_width:
             return None # Pattern complete
             
        # Alternate direction for each leg
        if self.current_leg % 2 == 0:
            # Even leg, fly +X
            x_pos = half_width
        else:
            # Odd leg, fly -X
            x_pos = -half_width
            
        self.current_leg += 1
        
        return Position(
            x=x_pos + search_area.x,
            y=y_pos + search_area.y,
            z=self.config.patrol_altitude
        )

# Factory function for composition
def create_lawnmower_search_strategy(config: LawnmowerConfig):
    return LawnmowerSearchStrategy(config)
