"""
HAL Implementation: SimulatedController
"""

import asyncio
import time
import math # <-- NEW
from typing import Optional, Dict, Any, List

# Import base class
from .hal import BaseFlightController

# Import component 10 definitions
from .vehicle_state import Telemetry, VehicleModeEnum, GPSStatusEnum

# --- FIX: Import explicit position types ---
from ..utils.position import LocalPosition, GlobalPosition
# --- End of FIX ---

# Import protocols (interfaces)
from ..g3_capability_plugins.strategies.base import Waypoint
from .sensors.cameras.base import BaseCamera, SimulatedCamera
from .sensors.lidar import Lidar, SimulatedLidar # <-- NEW
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
        
        # --- FIX: Store home location for GPS translation ---
        self._home_lat = self.config.get("start_lat", 44.5646)
        self._home_lon = self.config.get("start_lon", -123.2821)
        self._home_alt = self.config.get("start_alt", 0.0)
        # Cosine of latitude for longitude calculations
        self._lat_cos = math.cos(math.radians(self._home_lat))
        # --- End of FIX ---

        self._current_pos = Waypoint(0, 0, 0) # This is the internal LOCAL position
        self._target_pos: Optional[Waypoint] = None
        self._flight_speed_ms = self.config.get("flight_speed_ms", 10.0)
        self._battery = 100.0
        self._start_time = time.monotonic()
        self._telemetry_task: Optional[asyncio.Task] = None
        
        # Simulated hardware inventory
        self._cameras = {0: SimulatedCamera(0)}
        self._actuators = {0: SimulatedActuator()}
        self._lidars = {0: SimulatedLidar(0)} # <-- NEW
        
    async def connect(self):
        print(f"[SimulatedHAL] Connecting to simulated drone (Speed: {self._flight_speed_ms} m/s)...")
        print(f"[SimulatedHAL] Home (Origin): {self._home_lat}, {self._home_lon}")
        await asyncio.sleep(0.5)
        # Start the background task that updates telemetry
        self._telemetry_task = asyncio.create_task(self._update_telemetry())
        print("[SimulatedHAL] Connection successful.")

    async def detect_hardware(self) -> List[str]:
        """
        Simulates hardware detection.
        Returns the hardware list passed in from fleet_config.yaml.
        """
        print("[SimulatedHAL] Simulating hardware detection...")
        await asyncio.sleep(0.25) # Simulate a check
        
        # For simulation, the "actual" hardware is what we configured
        # in fleet_config.yaml, which was injected into self.config.
        detected = self.config.get('hardware', [])
        
        # We can also add default simulated hardware
        if "gps" not in detected:
            detected.append("gps")
        if "camera_thermal" not in detected and 0 in self._cameras:
             detected.append("camera_thermal")
        if "lidar" not in detected and 0 in self._lidars: # <-- NEW
             detected.append("lidar")
             
        return list(set(detected)) # Return unique list

    # --- NEW: Coordinate Translation Helper ---
    def _translate_local_to_global(self, local_pos: LocalPosition) -> GlobalPosition:
        """Converts local X/Y/Z meters to global Lat/Lon/Alt."""
        # X is East, Y is North
        lat = self._home_lat + (local_pos.y / 111111.0)
        lon = self._home_lon + (local_pos.x / (111111.0 * self._lat_cos))
        alt = self._home_alt + (-local_pos.z) # Z is Down, Alt is Up
        
        return GlobalPosition(lat=lat, lon=lon, alt=alt)
    # --- End of NEW ---

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
            
            # --- THIS IS THE FIX ---
            # 1. Simulate flight (Now with 3D vector math)
            if self._target_pos:
                # Calculate 3D vector to target
                dx = self._target_pos.x - self._current_pos.x
                dy = self._target_pos.y - self._current_pos.y
                dz = self._target_pos.z - self._current_pos.z
                
                # Calculate distance
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Check for arrival
                # We check if we're less than one "tick" away
                if distance < self._flight_speed_ms:
                    self._current_pos.x = self._target_pos.x
                    self._current_pos.y = self._target_pos.y
                    self._current_pos.z = self._target_pos.z
                    self._target_pos = None
                    print("[SimulatedHAL] Arrived at waypoint.")
                else:
                    # Calculate normalized direction vector (unit vector)
                    norm_x = dx / distance
                    norm_y = dy / distance
                    norm_z = dz / distance
                    
                    # Move one step (speed * direction)
                    self._current_pos.x += norm_x * self._flight_speed_ms
                    self._current_pos.y += norm_y * self._flight_speed_ms
                    self._current_pos.z += norm_z * self._flight_speed_ms
            # --- END OF FIX ---
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
            
            # --- FIX: Populate both Global and Local positions ---
            # 3. Create new position objects
            local_pos = LocalPosition(
                x=self._current_pos.x, 
                y=self._current_pos.y, 
                z=self._current_pos.z
            )
            global_pos = self._translate_local_to_global(local_pos)

            # 4. Create and publish telemetry
            new_telem = Telemetry(
                position_local=local_pos,     # <-- NEW
                position_global=global_pos,   # <-- NEW
                mode=new_mode,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=is_armed,
                attitude_yaw=self._current_pos.yaw # Pass along yaw
            )
            self.vehicle_state.update(new_telem)
            # --- End of FIX ---

    async def arm(self):
        print("[SimulatedHAL] Arming motors...")
        await asyncio.sleep(1)
        
        # --- FIX: Update Telemetry object construction ---
        local_pos = LocalPosition(self._current_pos.x, self._current_pos.y, self._current_pos.z)
        global_pos = self._translate_local_to_global(local_pos)
        self.vehicle_state.update(
            Telemetry(
                position_local=local_pos,
                position_global=global_pos,
                mode=VehicleModeEnum.ARMED,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=True
            )
        )
        # --- End of FIX ---
        print("[SimulatedHAL] Armed.")

    async def disarm(self):
        print("[SimulatedHAL] Disarming motors...")
        await asyncio.sleep(1)

        # --- FIX: Update Telemetry object construction ---
        local_pos = LocalPosition(self._current_pos.x, self._current_pos.y, self._current_pos.z)
        global_pos = self._translate_local_to_global(local_pos)
        self.vehicle_state.update(
            Telemetry(
                position_local=local_pos,
                position_global=global_pos,
                mode=VehicleModeEnum.DISARMED,
                gps_status=GPSStatusEnum.RTK_FIXED,
                battery_percent=self._battery,
                is_armed=False
            )
        )
        # --- End of FIX ---
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

    # --- THIS IS THE FIX ---
    async def set_target_waypoint(self, waypoint: Waypoint):
        """
        Sets a new target waypoint for the simulator.
        NON-BLOCKING.
        """
        if self.vehicle_state.mode != VehicleModeEnum.GUIDED:
            # Don't set target if not in the right mode
            return
            
        # The background _update_telemetry loop will see this new
        # _target_pos and start moving towards it on its next tick.
        self._target_pos = waypoint
        # No await, return immediately
    # --- END OF FIX ---

    async def land(self) -> bool:
        print("[SimulatedHAL] Landing...")
        await self.goto(Waypoint(self._current_pos.x, self._current_pos.y, 0))
        await self.disarm()
        print("[SimulatedHAL] Landed.")
        return True
        
    async def stop_movement(self):
        """Simulates an immediate brake by clearing the target position."""
        print("[SimulatedHAL] *** STOP MOVEMENT (BRAKE) ***")
        if self._target_pos:
            self._target_pos = None
            print("[SimulatedHAL] Active goto command cancelled.")

    def get_camera(self, camera_id: int) -> BaseCamera:
        return self._cameras[camera_id]
        
    def get_actuator_hardware(self, actuator_id: int) -> ActuatorHardware:
        return self._actuators[actuator_id]
        
    def get_lidar_sensor(self, sensor_id: int) -> Lidar:
        """Get an interface to a Lidar sensor."""
        return self._lidars[sensor_id]