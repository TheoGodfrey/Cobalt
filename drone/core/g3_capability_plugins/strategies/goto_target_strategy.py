"""
Component 7: Strategy (Go-To-Target Implementation)
A simple, single-waypoint strategy.
"""

import asyncio
from .base import BaseStrategy, Waypoint, VehicleState

class GoToTargetStrategy(BaseStrategy):
    """
    A simple strategy that always returns a single, predefined
    waypoint. Used for "go-to-location" or "deliver-to" tasks.
    
    Configuration (self.config) is expected to be a dict:
    {
        "target_x": 100.0,
        "target_y": 150.0,
        "altitude": -50.0
    }
    """
    def __init__(self, vehicle_state: VehicleState, config: dict):
        super().__init__(vehicle_state, config)
        
        # Load the target waypoint from the config
        target_x = config.get("target_x", 0.0)
        target_y = config.get("target_y", 0.0)
        altitude = config.get("altitude", -50.0)
        
        self._target_waypoint = Waypoint(x=target_x, y=target_y, z=altitude)
        
        print(f"[GoToTargetStrategy] Initialized. Target: ({target_x}, {target_y}, {altitude})")

    async def next_waypoint(self) -> Waypoint:
        """
        Returns the single target waypoint.
        """
        # This strategy only has one waypoint. It will return
        # the same one every time it is called.
        await asyncio.sleep(0.01) # Be a good async citizen
        return self._target_waypoint
