"""
Strategy interfaces (no inheritance, just protocol definition)
"""

# These are just for documentation - we use duck typing
class FlightStrategy:
    """Interface for flight strategies (composition-based)"""
    def get_next_position(self, drone, target_position):
        """Calculate next position to fly to"""
        pass

class SearchStrategy:
    """Interface for search strategies (composition-based)"""
    def get_next_position(self, drone, search_area, search_size):
        """Calculate next position to search"""
        pass