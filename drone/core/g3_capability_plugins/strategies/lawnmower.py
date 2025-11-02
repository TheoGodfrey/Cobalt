"""
Component 7: Strategy (Lawnmower Implementation)
A concrete implementation of BaseStrategy for a simple
grid-based lawnmower search pattern.
"""

import asyncio
from .base import BaseStrategy, Waypoint, VehicleState

class LawnmowerStrategy(BaseStrategy):
    """
    Generates waypoints for a simple N-S lawnmower pattern.
    
    Configuration (self.config) is expected to be a dict:
    {
        "origin_x": 0,
        "origin_y": 0,
        "width": 1000, # meters
        "height": 1000, # meters
        "step": 50, # meters
        "altitude": -50 # meters
    }
    """
    def __init__(self, vehicle_state: VehicleState, config: dict):
        super().__init__(vehicle_state, config)
        self._current_x = config.get("origin_x", 0)
        self._current_y = config.get("origin_y", 0)
        self._direction = 1 # 1 = North, -1 = South
        self._width = config.get("width", 1000)
        self._height = config.get("height", 1000)
        self._step = config.get("step", 50)
        self._altitude = config.get("altitude", -50)
        
        self._max_y = self._current_y + self._height
        self._min_y = self._current_y
        self._max_x = self._current_x + self._width
        
        print(f"[LawnmowerStrategy] Initialized. Area: {self._width}x{self._height}m, Step: {self._step}m")

    async def next_waypoint(self) -> Waypoint:
        """
        Calculates the next point in the lawnmower grid.
        """
        # Simulate calculation time
        await asyncio.sleep(0.1)
        
        if self._direction == 1: # Moving North
            self._current_y += self._height
            if self._current_y > self._max_y:
                self._current_y = self._max_y
        else: # Moving South
            self._current_y -= self._height
            if self._current_y < self._min_y:
                self._current_y = self._min_y
        
        # Create the waypoint
        wp = Waypoint(x=self._current_x, y=self._current_y, z=self._altitude)
        
        # Check if we need to move to the next "lane"
        if self._current_y == self._max_y or self._current_y == self._min_y:
            self._direction *= -1 # Reverse direction
            self._current_x += self._step # Move to next lane
            
            # If we are done with all lanes, restart
            if self._current_x > self._max_x:
                print("[LawnmowerStrategy] Pattern complete. Restarting...")
                self._current_x = self.config.get("origin_x", 0)
                self._current_y = self.config.get("origin_y", 0)
                self._direction = 1
        
        return wp
