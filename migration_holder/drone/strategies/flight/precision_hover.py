"""
Precision hover flight strategy - approach and hover at specific offset
"""

from core.position import Position

class PrecisionHoverFlightStrategy:
    """Precision hover - positions drone at exact offset above target"""
    
    def __init__(self, hover_altitude_offset: float = 2.0):
        self.name = "precision_hover"
        self.description = "Hover at precise altitude above target"
        self.hover_altitude_offset = hover_altitude_offset
    
    def get_next_position(self, drone, target_position) -> Position:
        """
        Get position to hover at - same x,y as target, 
        but at specified altitude above it
        """
        return Position(
            target_position.x,
            target_position.y,
            target_position.z + self.hover_altitude_offset
        )

# Factory function for composition
def create_precision_hover_flight_strategy(config): # <-- FIX: Added config
    # Use config if provided, otherwise default
    offset = config.altitude_offset if config else 2.0
    return PrecisionHoverFlightStrategy(hover_altitude_offset=offset)