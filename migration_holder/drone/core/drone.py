"""
Drone interface and Hardware Abstraction Layer (HAL) for flight controllers.
Includes Base, Simulated, and MAVLink implementations.
"""

import asyncio
import time
import json
import random
import math # <-- FIX: Added missing import
from abc import ABC, abstractmethod
from pydantic import BaseModel # <-- FIX: Was dataclass
from .position import Position

# --- Telemetry Dataclass (CHANGED to Pydantic BaseModel) ---

class Telemetry(BaseModel): # <-- FIX: Was @dataclass
    """Holds the complete state of the drone. (pydantic model for .model_dump())"""
    position: Position = Position(0, 0, 0)
    attitude_roll: float = 0.0
    attitude_pitch: float = 0.0
    attitude_yaw: float = 0.0
    battery: float = 100.0
    is_connected: bool = False
    # CHANGED: state is now the *vehicle* state, not the mission phase
    state: str = "DISARMED"  # e.g., DISARMED, ARMED, GUIDED, LOITER, MANUAL, LAND
    led_color: str = "off"
    last_heartbeat: float = 0.0

# --- BaseFlightController Interface (No Change) ---

class BaseFlightController(ABC):
    """Abstract interface for a flight controller (real or simulated)."""
    
    @abstractmethod
    async def connect(self) -> bool:
        pass
    @abstractmethod
    async def disconnect(self) -> None:
        pass
    @abstractmethod
    async def takeoff(self, altitude: float) -> bool:
        pass
    @abstractmethod
    async def go_to(self, position: Position) -> bool:
        pass
    @abstractmethod
    async def hover(self) -> bool:
        pass
    @abstractmethod
    async def land(self) -> bool:
        pass
    @abstractmethod
    async def set_led(self, color: str) -> None:
        pass
    @abstractmethod
    async def get_telemetry(self) -> Telemetry:
        pass

# --- Drone Class (FIXED) ---

class Drone:
    """
    High-level Drone class.
    Manages state and delegates flight commands to a controller (HAL).
    """
    def __init__(self, controller: BaseFlightController, drone_id: str = "drone_0"):
        self.id = drone_id
        self.controller = controller
        self.telemetry = Telemetry()
        self.health_history = []

    async def connect(self) -> bool:
        success = await self.controller.connect()
        self.telemetry.is_connected = success
        return success

    async def disconnect(self) -> None:
        await self.controller.disconnect()
        self.telemetry.is_connected = False

    async def takeoff(self, altitude: float) -> bool:
        # We no longer set telemetry.state here.
        # The controller's get_telemetry() is the source of truth.
        return await self.controller.takeoff(altitude)

    async def go_to(self, position: Position) -> bool:
        return await self.controller.go_to(position)

    async def hover(self) -> bool:
        return await self.controller.hover()

    async def land(self) -> bool:
        return await self.controller.land()

    async def set_led(self, color: str):
        await self.controller.set_led(color)
        # self.telemetry.led_color = color # Let get_telemetry handle this

    async def update_telemetry(self) -> None:
        """Poll the controller for the latest state."""
        self.telemetry = await self.controller.get_telemetry()
        self.telemetry.last_heartbeat = time.time()
        self.record_health()

    def is_healthy(self, battery_threshold: float, heartbeat_threshold: float) -> bool: # <-- FIX
        """Check if drone is healthy based on latest telemetry."""
        return (
            self.telemetry.is_connected and
            self.telemetry.battery > battery_threshold and # <-- FIX
            (time.time() - self.telemetry.last_heartheartbeat_threshold) # <-- FIX
        )

    def record_health(self):
        """Record current health snapshot."""
        self.health_history.append(self.telemetry)
        if len(self.health_history) > 10:
            self.health_history.pop(0)

# --- SimulatedFlightController Implementation (Updated) ---

class SimulatedFlightController(BaseFlightController):
    """
    Simulation of the flight controller.
    """
    def __init__(self):
        self._telemetry = Telemetry()
        self._telemetry.state = "DISARMED"
        print("[SimulatedController] Initialized.")

    async def connect(self) -> bool:
        print("[SimulatedController] Connecting...")
        await asyncio.sleep(0.5)
        self._telemetry.is_connected = True
        self._telemetry.state = "ARMED" # MAVLink state for "on, not flying"
        self._telemetry.last_heartbeat = time.time()
        print("[SimulatedController] Connected.")
        return True

    async def disconnect(self) -> None:
        print("[SimulatedController] Disconnecting...")
        await asyncio.sleep(0.1)
        self._telemetry.is_connected = False
        self._telemetry.state = "DISARMED"
        print("[SimulatedController] Disconnected.")

    async def takeoff(self, altitude: float) -> bool:
        print(f"[SimulatedController] Taking off to {altitude}m...")
        self._telemetry.state = "TAKING_OFF"
        await asyncio.sleep(2.0)
        self._telemetry.position.z = altitude
        self._telemetry.state = "GUIDED" # Using a MAVLink-like state
        print(f"[SimulatedController] At altitude {self._telemetry.position.z}m.")
        return True

    async def go_to(self, position: Position) -> bool:
        current_pos = self._telemetry.position
        dist = current_pos.distance_to(position)
        print(f"[SimulatedController] Flying from {current_pos} to {position} ({dist:.1f}m)...")
        self._telemetry.state = "GUIDED"
        
        # Simulate yaw (drone turns to face target)
        self._telemetry.attitude_yaw = math.degrees(math.atan2(position.y - current_pos.y, position.x - current_pos.x))
        # Simulate slight roll/pitch during movement
        self._telemetry.attitude_pitch = random.uniform(-5.0, 5.0)

        await asyncio.sleep(dist / 10.0) # Simulate flight time (10 m/s)
        self._telemetry.position = position
        self._telemetry.state = "LOITER" # MAVLink state for "hovering"
        self._telemetry.attitude_pitch = 0.0 # Level out
        print(f"[SimulatedController] Arrived at {position}.")
        return True

    async def hover(self) -> bool:
        print("[SimulatedController] Hovering.")
        self._telemetry.state = "LOITER"
        self._telemetry.attitude_pitch = random.uniform(-0.5, 0.5) # Slight hover movement
        self._telemetry.attitude_roll = random.uniform(-0.5, 0.5)
        await asyncio.sleep(0.1)
        return True

    async def land(self) -> bool:
        print("[SimulatedController] Landing...")
        self._telemetry.state = "LANDING"
        await asyncio.sleep(2.0)
        self._telemetry.position.z = 0
        self._telemetry.state = "ARMED" # Landed but not disarmed
        print("[SimulatedController] Landed.")
        return True

    async def set_led(self, color: str):
        print(f"[SimulatedController] LED set to {color.upper()}")
        self._telemetry.led_color = color
        await asyncio.sleep(0.01)

    async def get_telemetry(self) -> Telemetry:
        if self._telemetry.state not in ["DISARMED", "ARMED"]:
            self._telemetry.battery -= 0.01
        
        # --- NEW: Simulate Local Operator Takeover ---
        if self._telemetry.state != "MANUAL" and random.random() < 0.005:
            print("[SimulatedController] !!! LOCAL OPERATOR TOOK CONTROL (MANUAL) !!!")
            self._telemetry.state = "MANUAL"
        elif self._telemetry.state == "MANUAL" and random.random() < 0.01:
            print("[SimulatedController] !!! LOCAL OPERATOR RELEASED CONTROL (GUIDED) !!!")
            self._telemetry.state = "GUIDED"
        # --- End Simulation ---

        self._telemetry.last_heartbeat = time.time()
        # Add slight hover drift to attitude
        if self._telemetry.state == "LOITER":
             self._telemetry.attitude_pitch = random.uniform(-0.5, 0.5)
             self._telemetry.attitude_roll = random.uniform(-0.5, 0.5)
        return self._telemetry.model_copy() # <-- FIX: Use model_copy() for Pydantic

# --- MavlinkFlightController Implementation (Updated) ---

class MavlinkController(BaseFlightController):
    """
    Hardware implementation of the flight controller using MAVLink.
    (Simulated implementation of real-world logic)
    """
    def __init__(self, connection_string: str = "udp:127.0.0.1:14550"):
        # e.g., from dronekit import connect, VehicleMode
        # self.vehicle = None
        self.connection_string = connection_string
        self._telemetry = Telemetry()
        self._telemetry.state = "DISARMED"
        print(f"[MavlinkController] Initialized. Will connect to {connection_string}")

    async def connect(self) -> bool:
        print(f"[MavlinkController] Connecting to {self.connection_string}...")
        # e.g., self.vehicle = connect(self.connection_string, wait_ready=True, timeout=10)
        await asyncio.sleep(1.0)
        self._telemetry.is_connected = True
        self._telemetry.state = "ARMED" # Assume connection success
        self._telemetry.last_heartbeat = time.time()
        print("[MavlinkController] Vehicle connected.")
        return True

    async def disconnect(self) -> None:
        print("[MavlinkController] Disconnecting...")
        # e.g., self.vehicle.close()
        await asyncio.sleep(0.1)
        self._telemetry.is_connected = False
        self._telemetry.state = "DISARMED"
        print("[MavlinkController] Disconnected.")

    async def takeoff(self, altitude: float) -> bool:
        print("[MavlinkController] Setting mode to GUIDED...")
        # e.g., self.vehicle.mode = VehicleMode("GUIDED")
        print("[MavlinkController] Arming vehicle...")
        # e.g., self.vehicle.armed = True
        await asyncio.sleep(1.0)
        print(f"[MavlinkController] Sending takeoff command to {altitude}m...")
        # e.g., self.vehicle.simple_takeoff(altitude)
        await asyncio.sleep(3.0)
        # self._telemetry.position.z = altitude # Let get_telemetry handle this
        print(f"[MavlinkController] Takeoff complete.")
        return True

    async def go_to(self, position: Position) -> bool:
        print(f"[MavlinkController] Setting mode to GUIDED and flying to {position}...")
        # e.g., self.vehicle.mode = VehicleMode("GUIDED")
        # e.g., loc = LocationGlobalRelative(position.x, position.y, position.z)
        # e.g., self.vehicle.simple_goto(loc)
        await asyncio.sleep(2.0)
        print(f"[MavlinkController] Arrived at {position}.")
        return True

    async def hover(self) -> bool:
        print("[MavlinkController] Setting mode to LOITER/HOVER...")
        self._telemetry.state = "LOITER" # <-- FIX: Set MAVLink state
        # e.g., self.vehicle.mode = VehicleMode("LOITER")
        await asyncio.sleep(0.2)
        return True

    async def land(self) -> bool:
        print("[MavlinkController] Setting mode to LAND...")
        # e.g., self.vehicle.mode = VehicleMode("LAND")
        await asyncio.sleep(3.0)
        self._telemetry.state = "ARMED" # <-- FIX: Set MAVLink state
        print("[MavlinkController] Landed and disarmed.")
        return True

    async def set_led(self, color: str):
        print(f"[MavlinkController] (SIMULATED) Set LED to {color.upper()}")
        self._telemetry.led_color = color
        await asyncio.sleep(0.01)

    async def get_telemetry(self) -> Telemetry:
        # --- THIS IS THE KEY CHANGE ---
        # In a real implementation:
        # if self.vehicle:
        #    # Check for manual takeover
        #    if self.vehicle.mode.name == "MANUAL":
        #        self._telemetry.state = "MANUAL"
        #    elif self.vehicle.mode.name == "GUIDED":
        #        self._telemetry.state = "GUIDED"
        #    # ...etc.
        #
        #    loc = self.vehicle.location.global_relative_frame
        #    # NOTE: Real MAVLink uses Lat/Lon. You would need a conversion.
        #    self._telemetry.position = Position(x=loc.lat, y=loc.lon, z=loc.alt)
        #    att = self.vehicle.attitude
        #    self._telemetry.attitude_roll = math.degrees(att.roll)
        #    self._telemetry.attitude_pitch = math.degrees(att.pitch)
        #    self._telemetry.attitude_yaw = math.degrees(att.yaw)
        #    self._telemetry.battery = self.vehicle.battery.level
        #    self._telemetry.is_connected = True
        # else:
        #    self._telemetry.is_connected = False
        
        # --- Simulated MAVLink Implementation ---
        if self._telemetry.state not in ["DISARMED", "ARMED"]:
            self._telemetry.battery -= 0.02
        
        # Simulate small position/attitude changes
        self._telemetry.position.x += 0.01
        self._telemetry.attitude_yaw = (self._telemetry.attitude_yaw + 0.5) % 360
        self._telemetry.last_heartbeat = time.time()
        
        return self._telemetry.model_copy()

