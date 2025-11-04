"""
Component 10: Vehicle State
Physical vehicle status monitor.

This component tracks the "physical reality" of the drone: its position,
battery, and hardware mode (e.g., MANUAL, GUIDED).

This is critically separate from MissionState (G2), which tracks
"logical intent" (e.g., SEARCHING, DELIVERING).
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Callable, Set

# Re-using the Waypoint definition from G3 strategies
from ..g3_capability_plugins.strategies.base import Waypoint

class VehicleModeEnum(Enum):
    """Possible flight controller modes."""
    DISARMED = auto()
    ARMED = auto()
    MANUAL = auto()     # Full manual pilot control
    GUIDED = auto()     # Accepting autonomous commands
    RTL = auto()        # Autonomous Return-to-Launch
    LAND = auto()       # Autonomous Land
    EMERGENCY = auto()  # Failsafe mode

class GPSStatusEnum(Enum):
    """GPS fix quality."""
    NO_FIX = auto()
    FIX_2D = auto()
    FIX_3D = auto()
    DGPS = auto()
    RTK_FIXED = auto()

@dataclass
class Telemetry:
    """
    A snapshot of the vehicle's physical state.
    This is what the HAL (G4) produces and VehicleState (G4) consumes.
    """
    timestamp: float = field(default_factory=time.monotonic)
    position: Waypoint = field(default_factory=lambda: Waypoint(0,0,0))
    mode: VehicleModeEnum = VehicleModeEnum.DISARMED
    gps_status: GPSStatusEnum = GPSStatusEnum.NO_FIX
    battery_percent: float = 100.0
    is_armed: bool = False
    wind_speed: float = 0.0
    wind_direction: float = 0.0
    
    # --- FIX 1.1: Added attitude fields for geolocation ---
    # These are required by core/utils/navigation.py
    # 
    attitude_roll: float = 0.0  # In degrees
    attitude_pitch: float = 0.0 # In degrees
    attitude_yaw: float = 0.0   # In degrees
    # --- End of FIX 1.1 ---

# Type alias for a listener: Callable[[Telemetry], None]
TelemetryChangeListener = Callable[[Telemetry], None]

class VehicleState:
    """
    Manages the physical vehicle state and notifies listeners of changes.
    
    This class is owned and updated by the HAL (Component 9).
    It is observed by the SafetyMonitor (Component 12) and
    MissionController (Component 3).
    """
    
    def __init__(self):
        self._current_telemetry: Telemetry = Telemetry()
        self._listeners: Set[TelemetryChangeListener] = set()

    def add_listener(self, listener: TelemetryChangeListener):
        """Register a callback for telemetry updates."""
        self._listeners.add(listener)

    def remove_listener(self, listener: TelemetryChangeListener):
        """Unregister a telemetry update listener."""
        self._listeners.discard(listener)

    def _notify_listeners(self):
        """Notify all listeners of the new telemetry data."""
        for listener in self._listeners:
            try:
                listener(self._current_telemetry)
            except Exception as e:
                print(f"[VehicleState] Error in listener {listener}: {e}")

    def update(self, new_telemetry: Telemetry):
        """
        Called by the HAL to provide a new state snapshot.
        """
        # TODO: Could add logic here to check for significant changes
        # before notifying all listeners, to reduce noise.
        self._current_telemetry = new_telemetry
        self._notify_listeners()

    # --- Property Accessors ---
    # These provide clean read-only access for other components.

    @property
    def current(self) -> Telemetry:
        """Get the latest telemetry snapshot."""
        return self._current_telemetry

    @property
    def position(self) -> Waypoint:
        """Get current drone position."""
        return self._current_telemetry.position
    
    @property
    def mode(self) -> VehicleModeEnum:
        """Get current flight controller mode."""
        return self._current_telemetry.mode
        
    @property
    def battery_percent(self) -> float:
        """Get current battery percentage."""
        return self._current_telemetry.battery_percent

    @property
    def is_safe_to_fly(self) -> bool:
        """Example of a derived state property."""
        return (
            self._current_telemetry.gps_status in (GPSStatusEnum.FIX_3D, GPSStatusEnum.RTK_FIXED) and
            self._current_telemetry.battery_percent > 25
        )

    def __str__(self) -> str:
        return (
            f"VehicleState(Mode: {self.mode.name}, "
            f"Batt: {self.battery_percent:.1f}%, "
            f"Pos: {self.position.x}, {self.position.y}, {self.position.z})"
        )

