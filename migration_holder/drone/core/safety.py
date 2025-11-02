"""
Safety layer for collision avoidance.
Implements the Decorator pattern by wrapping a BaseFlightController.
"""
import asyncio
from .drone import BaseFlightController, Telemetry
from .position import Position

# --- FIX: Added forward-ref import for MqttClient type hint ---
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .comms import MqttClient

class StubObstacleSensor:
    """
    A STUB for a 3D sensor suite (e.g., LiDAR, Stereo Camera).
    In a real system, this class would be complex.
    """
    def __init__(self):
        print("[StubObstacleSensor] Initialized.")
        # In reality, you would connect to the sensor hardware here.
    
    async def is_path_clear(self, start: Position, end: Position) -> bool:
        """STUB: Check if the direct path is clear."""
        print("[CollisionAvoider] Checking if path is clear...")
        await asyncio.sleep(0.05) # Simulate sensor check
        # STUB: Always return True for now
        is_clear = True
        if not is_clear:
            print("[CollisionAvoider] OBSTACLE DETECTED on direct path.")
        return is_clear

    async def calculate_safe_path(self, start: Position, end: Position) -> list[Position]:
        """STUB: Calculate a safe path (e.g., using A*)."""
        print("[CollisionAvoider] STUB: Calculating safe alternative path...")
        await asyncio.sleep(0.2) # Simulate path planning
        # STUB: Return a simple "go up, over, and down" path
        safe_path = [
            Position(start.x, start.y, start.z + 10.0), # Go up 10m
            Position(end.x, end.y, end.z + 10.0),     # Go over
            end                                       # Go to destination
        ]
        return safe_path

class CollisionAvoider(BaseFlightController):
    """
    Decorator for a flight controller that adds a collision avoidance layer.
    It intercepts 'go_to' commands and checks them for safety.
    """
    
    # --- FIX: Added mqtt_client argument to match main.py ---
    def __init__(self, 
                 wrapped_controller: BaseFlightController, 
                 sensor: StubObstacleSensor, 
                 mqtt_client: "MqttClient"):
        print(f"[CollisionAvoider] Wrapping {type(wrapped_controller).__name__}.")
        self.controller = wrapped_controller
        self.sensor = sensor
        self.mqtt = mqtt_client # <-- ADDED
        self.last_obstacle_warning = 0

    # --- Pass-through methods ---
    
    async def connect(self) -> bool:
        return await self.controller.connect()

    async def disconnect(self) -> None:
        return await self.controller.disconnect()

    async def takeoff(self, altitude: float) -> bool:
        # Takeoff is assumed to be in a clear area (or use a different check)
        return await self.controller.takeoff(altitude)

    async def hover(self) -> bool:
        return await self.controller.hover()

    async def land(self) -> bool:
        # Land is assumed to be in a clear area (or use a different check)
        return await self.controller.land()

    async def set_led(self, color: str) -> None:
        return await self.controller.set_led(color)

    async def get_telemetry(self) -> Telemetry:
        return await self.controller.get_telemetry()
    
    # --- Intercepted method ---
    
    async def go_to(self, position: Position) -> bool:
        """
        Intercepts the go_to command to check for safety.
        """
        print(f"[CollisionAvoider] Intercepted go_to({position})")
        
        current_telemetry = await self.get_telemetry()
        current_pos = current_telemetry.position

        if await self.sensor.is_path_clear(current_pos, position):
            # Path is clear, pass command directly to wrapped controller
            print("[CollisionAvoider] Path is clear. Executing direct flight.")
            return await self.controller.go_to(position)
        else:
            # Path is blocked, calculate a safe path
            print("[CollisionAvoider] Path blocked. Calculating alternative route...")
            
            # --- FIX: Use the MQTT client ---
            if (asyncio.get_event_loop().time() - self.last_obstacle_warning > 5.0):
                await self.mqtt.publish(f"fleet/event/{self.controller.id}", {
                    "type": "WARNING",
                    "data": {"message": "Obstacle detected, recalculating path."}
                })
                self.last_obstacle_warning = asyncio.get_event_loop().time()
            
            safe_waypoints = await self.sensor.calculate_safe_path(current_pos, position)
            
            if not safe_waypoints:
                print("[CollisionAvoider] ERROR: Could not find a safe path.")
                return False
                
            print(f"[CollisionAvoider] Executing alternative path with {len(safe_waypoints)} waypoints.")
            for i, wp in enumerate(safe_waypoints):
                print(f"[CollisionAvoider] Flying to safe waypoint {i+1}/{len(safe_waypoints)}: {wp}")
                success = await self.controller.go_to(wp)
                if not success:
                    print(f"[CollisionAvoider] ERROR: Failed to fly to safe waypoint {wp}.")
                    return False
            
            print("[CollisionAvoider] Alternative path complete.")
            return True