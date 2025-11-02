"""
HAL Implementation: SimulatedController
"""

import asyncio
import time
from typing import Optional, Dict, Any

# Import base class
from .hal import BaseFlightController

# Import component 10 definitions
from .vehicle_state import Telemetry, VehicleModeEnum, GPSStatusEnum

# Import protocols (interfaces)
from ..g3_capability_plugins.strategies.base import Waypoint
from .sensors.cameras.base import BaseCamera, SimulatedCamera
from ..g3_capability_plugins.actuators.base import ActuatorHardware

class SimulatedActuator(ActuatorHardware):
    """Simulated hardware for a payload dropper."""
    async def set_angle(self, angle: float):
        print(f"  [SimHW] Servo angle set to {angle} deg")
        await asyncio.sleep(0.1)
    
    async def release(self):
        print(f"  [SimHW] *** Dropper mechanism RELEASED ***")
        await asyncio.sleep(0.1)

class SimulatedController(BaseFlightController):
    """
    A concrete implementation of the HAL for use in simulation.
    It simulates flight physics, battery drain, and sensors.
    """
    
    def __init__(self, config: Dict[str, Any]):
        # Call super() with config (fixes Bug #7 compatibility)
        super().__init__(config) 
        
        # FIX for Bug #8: self.config is now set by the base class
        # We can now safely access it.
        
        self._current_pos = Waypoint(0, 0, 0)
        self._target_pos: Optional[Waypoint] = None
        self._flight_speed_ms = self.config.get("flight_speed_ms", 10.0)
        self._battery = 100.0
        self._start_time = time.monotonic()
        self._telemetry_task: Optional[asyncio.Task] = None
        
        # Simulated hardware inventory
        self._cameras = {0: SimulatedCamera(0)}
        self._actuators = {0: SimulatedActuator()}
        
    async def connect(self):
        print(f"[SimulatedHAL] Connecting to simulated drone (Speed: {self._flight_speed_ms} m/s)...")
        await asyncio.sleep(0.5)
        # Start the background task that updates telemetry
        self._telemetry_task = asyncio.create_task(self._update_telemetry())
        print("[SimulatedHAL] Connection successful.")

    async def _update_telemetry(self):
        """A background task to simulate the drone's state over time."""
        while True:
            await asyncio.sleep(1.0) # Update telemetry 1 time per second
            
            # Default to GUIDED if we're not DISARMED
            new_mode = self.vehicle_state.mode
            if new_mode == VehicleModeEnum.DISARMED:
                is_armed = False
            else:
                is_armed = True
                new_mode = VehicleModeEnum.GUIDED # Default active mode
            
            # 1. Simulate flight
            if self._target_pos:
                # Simplified 1D movement for now
                dx = self._target_pos.x - self._current_pos.x
                
                if abs(dx) < self._flight_speed_ms:
                    self._current_pos = self._target_pos
                    self._target_pos = None
                    print("[SimulatedHAL] Arrived at waypoint.")
                else:
                    move = self._flight_speed_ms if dx > 0 else -self._flight_speed_ms
                    self._current_pos.x += move
            else:
                if is_armed:
                    new_mode = VehicleModeEnum.ARMED # Loitering
            
            # 2. Simulate battery drain
            # --- FIX for Bug #19 ---
            # Only drain battery if motors are active (ARMED or GUIDED)
            active_modes = (VehicleModeEnum.ARMED, VehicleModeEnum.GUIDED)
            if new_mode in active_modes:
                # Drains 1% every 20 seconds (approx 33 min flight time)
                self._battery -= 1.0 / 20.0 
                if self._battery < 0: self._battery = 0
            
            # 3. Create and publish telemetry
            new_telem = Telemetry(
                position=self._current_pos,
                mode=new_mode,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=is_armed
            )
            self.vehicle_state.update(new_telem)

    async def arm(self):
        print("[SimulatedHAL] Arming motors...")
        await asyncio.sleep(1)
        self.vehicle_state.update(
            Telemetry(
                position=self._current_pos,
                mode=VehicleModeEnum.ARMED,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=True
            )
        )
        print("[SimulatedHAL] Armed.")

    async def disarm(self):
        print("[SimulatedHAL] Disarming motors...")
        await asyncio.sleep(1)
        self.vehicle_state.update(
            Telemetry(
                position=self._current_pos,
                mode=VehicleModeEnum.DISARMED,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=False
            )
        )
        print("[SimulatedHAL] Disarmed.")

    async def goto(self, waypoint: Waypoint) -> bool:
        if self.vehicle_state.mode == VehicleModeEnum.MANUAL:
            print("[SimulatedHAL] GOTO command rejected: Vehicle in MANUAL mode.")
            return False
            
        print(f"[SimulatedHAL] Flying to waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})")
        self._target_pos = waypoint
        
        # Wait until we arrive
        while self._target_pos is not None:
            await asyncio.sleep(0.5)
            
        return True

    async def land(self) -> bool:
        print("[SimulatedHAL] Landing...")
        await self.goto(Waypoint(self._current_pos.x, self._current_pos.y, 0))
        await self.disarm()
        print("[SimulatedHAL] Landed.")
        return True

    def get_camera(self, camera_id: int) -> BaseCamera:
        return self._cameras[camera_id]
        
    def get_actuator_hardware(self, actuator_id: int) -> ActuatorHardware:
        return self._actuators[actuator_id]