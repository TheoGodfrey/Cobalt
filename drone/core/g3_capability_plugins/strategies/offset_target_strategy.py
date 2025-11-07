import asyncio
from .base import BaseStrategy, Waypoint, VehicleState

class OffsetTargetStrategy(BaseStrategy):
    """
    A simple strategy that returns a single waypoint, calculated
    as an offset from a given target coordinate.
    
    Used for "deliver-next-to" or "crash-near" tasks.
    
    Config:
    {
        "target_x": 100.0,
        "target_y": 150.0,
        "altitude": -2.0,  // Low altitude for a water landing
        "offset_x": 5.0,   // e.g., 5 meters East
        "offset_y": 0.0    // e.g., 0 meters North
    }
    """
    def __init__(self, vehicle_state: VehicleState, config: dict):
        super().__init__(vehicle_state, config)
        
        # Load the target coordinate from the config
        target_x = config.get("target_x", 0.0)
        target_y = config.get("target_y", 0.0)
        altitude = config.get("altitude", -2.0) # Default to 2m above
        
        # Load the offset from the config
        offset_x = config.get("offset_x", 5.0) # Default 5m X offset
        offset_y = config.get("offset_y", 0.0)
        
        # Calculate the final "splash point"
        splash_x = target_x + offset_x
        splash_y = target_y + offset_y
        
        self._target_waypoint = Waypoint(x=splash_x, y=splash_y, z=altitude)
        
        print(f"[OffsetTargetStrategy] Initialized. Target: ({splash_x}, {splash_y}, {altitude})")

    async def next_waypoint(self) -> Waypoint:
        """
        Returns the single, calculated offset waypoint.
        """
        await asyncio.sleep(0.01)
        return self._target_waypoint