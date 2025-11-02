"""
Direct flight strategy - fly straight to target
"""

from core.drone import Position

class DirectFlightStrategy:
    """Direct flight algorithm - fly straight to target"""
    
    def __init__(self):
        self.name = "direct"
        self.description = "Fly directly to target position"
    
    def get_next_position(self, drone, target_position) -> Position:
        """Get the next position to fly to (directly to target)"""
        return target_position

# Factory function for composition
def create_direct_flight_strategy(config=None): # <-- FIX: Added config=None
    return DirectFlightStrategy()