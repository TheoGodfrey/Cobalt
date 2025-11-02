"""
Random search algorithm - simple random pattern
"""

from core.drone import Position
import random

class RandomSearchStrategy:
    """Random search algorithm - random pattern for search"""
    
    def __init__(self):
        self.name = "random"
        self.description = "Random search pattern"
    
    def get_next_position(self, drone, search_area, search_size):
        """Get the next position to search (random pattern)"""
        # Handle both dict and Position object for search_area
        if isinstance(search_area, dict):
            area_x = search_area['x']
            area_y = search_area['y']
        else:
            area_x = search_area.x
            area_y = search_area.y
        
        x = random.uniform(
            area_x - search_size/2, 
            area_x + search_size/2
        )
        y = random.uniform(
            area_y - search_size/2, 
            area_y + search_size/2
        )
        
        # Return position at search altitude (15m)
        return Position(x, y, 15.0)

# Factory function for composition
def create_random_search_strategy(config=None): # <-- FIX: Added config=None
    return RandomSearchStrategy()