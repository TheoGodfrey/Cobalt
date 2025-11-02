"""
Vertical ascent search algorithm - scan while climbing
"""

from core.position import Position

class VerticalAscentSearchStrategy:
    """Vertical ascent search - climb vertically while scanning"""
    
    def __init__(self, max_altitude: float = 120.0, step_size: float = 5.0):
        self.name = "vertical_ascent"
        self.description = "Climb vertically while scanning"
        self.max_altitude = max_altitude
        self.step_size = step_size  # Meters to climb each step
        self.current_altitude = 0.0
    
    def get_next_position(self, drone, search_area, search_size):
        """
        Get next position - climb vertically from current position.
        search_area parameter is used for x,y home position.
        """
        # Stay at home x,y, just increase altitude
        if isinstance(search_area, dict):
            home_x = search_area['x']
            home_y = search_area['y']
        else:
            home_x = search_area.x
            home_y = search_area.y
        
        # Increment altitude
        self.current_altitude += self.step_size
        
        # Cap at max altitude
        if self.current_altitude > self.max_altitude:
            self.current_altitude = self.max_altitude
        
        return Position(home_x, home_y, self.current_altitude)

# Factory function for composition
def create_vertical_ascent_search_strategy(config): # <-- FIX: Added config
    # Use config if provided, otherwise default
    max_alt = config.max_altitude if config else 120.0
    step = config.step_size if config else 5.0
    return VerticalAscentSearchStrategy(max_altitude=max_alt, step_size=step)