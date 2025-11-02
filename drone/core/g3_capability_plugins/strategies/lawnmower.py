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
    This implementation generates the *corners* of the pattern.
    
    Configuration (self.config) is expected to be a dict:
    {
        "origin_x": 0,
        "origin_y": 0,
        "width": 1000, # meters (total width of area on X-axis)
        "height": 1000, # meters (total height of area on Y-axis)
        "step": 50, # meters (distance between N-S legs)
        "altitude": -50 # meters
    }
    """
    def __init__(self, vehicle_state: VehicleState, config: dict):
        super().__init__(vehicle_state, config)
        self._origin_x = config.get("origin_x", 0)
        self._origin_y = config.get("origin_y", 0)
        self._width = config.get("width", 1000)
        self._height = config.get("height", 1000)
        self._step = config.get("step", 50)
        self._altitude = config.get("altitude", -50)
        
        self._max_y = self._origin_y + self._height
        self._min_y = self._origin_y
        self._max_x = self._origin_x + self._width
        
        self._current_x = self._origin_x
        self._current_y = self._origin_y
        
        # 1 = North (moving towards +Y), -1 = South (moving towards -Y)
        self._direction = 1
        
        # 0 = moving along Y (leg), 1 = moving along X (step)
        self._state = 0
        
        print(f"[LawnmowerStrategy] Initialized. Area: {self._width}x{self._height}m, Step: {self._step}m")

    async def next_waypoint(self) -> Waypoint:
        """
        Calculates the next corner point in the lawnmower grid.
        """
        # Simulate calculation time
        await asyncio.sleep(0.1)
        
        # --- FIX: Re-implemented lawnmower logic ---
        
        # Check if we are done with all lanes
        if self._current_x > self._max_x:
            print("[LawnmowerStrategy] Pattern complete. Restarting...")
            self._current_x = self._origin_x
            self._current_y = self._origin_y
            self._direction = 1
            self._state = 0 # Start by moving along Y

        if self._state == 0:
            # --- State 0: Move along the Y-axis (North or South leg) ---
            
            # Set target Y to the end of the current leg
            if self._direction == 1:
                self._current_y = self._max_y # Move to North end
            else:
                self._current_y = self._min_y # Move to South end
            
            # Create the waypoint for the end of this leg
            wp = Waypoint(x=self._current_x, y=self._current_y, z=self._altitude)
            
            # Next, we will step over (state 1)
            self._state = 1 
            return wp
            
        else:
            # --- State 1: Step over along the X-axis ---
            
            # Move to the next lane
            self._current_x += self._step
            
            # If this step takes us out of bounds, restart
            if self._current_x > self._max_x:
                print("[LawnmowerStrategy] Pattern complete. Restarting...")
                self._current_x = self._origin_x
                self._current_y = self._origin_y
                self._direction = 1
                self._state = 0
                # Return the home position as the last point
                return Waypoint(x=self._origin_x, y=self._origin_y, z=self._altitude)

            # Create the waypoint for the start of the next leg
            wp = Waypoint(x=self._current_x, y=self._current_y, z=self._altitude)

            # Next, we will move along Y (state 0)
            self._state = 0
            # And reverse direction for the next leg
            self._direction *= -1
            return wp
        # --- End of Fix ---