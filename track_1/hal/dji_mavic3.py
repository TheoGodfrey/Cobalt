# shared/hal/dji_mavic3.py
# track1_shipping/hal.py
#
# Contains the DJIMavic3 class, moved from shared/hal/dji_mavic3.py
# All imports are standard libraries.

import time
import random
from typing import Dict, Any, Tuple

class DJIMavic3:
    """
    [FULL STUB] Represents the DJI Drone HAL.
    This simulates all methods needed by the mission, remote, and safety.
    """
    def __init__(self, connection_string):
        print(f"[HAL] Connecting to drone at {connection_string}...")
        self._current_pos = (51.5074, -0.1278) # (lat, lon)
        self._current_altitude = 0.0 # meters
        self._home_pos = self._current_pos
        self._battery_percent = 100.0
        self._is_armed = False
        print("[HAL] Drone connected.")

    # --- Core Flight Commands ---
    
    def arm(self):
        print("[HAL] Drone ARMED.")
        self._is_armed = True
        time.sleep(0.5)

    def disarm(self):
        print("[HAL] Drone DISARMED.")
        self._is_armed = False
        time.sleep(0.5)

    def takeoff(self):
        print("[HAL] Drone TAKING OFF.")
        self._current_altitude = 10.0
        self._battery_percent -= 0.1 # Simulate battery use
        time.sleep(1)

    def land(self, descent_rate=1.0):
        print(f"[HAL] Landing at {descent_rate} m/s...")
        # Simulate landing flight
        self.goto(self._current_pos, altitude=0.0)
        self._current_altitude = 0.0
        print("[HAL] Landed.")

    def goto(self, position: Tuple[float, float], altitude: float = None, speed: float = None):
        lat, lon = position
        if altitude is None:
            altitude = self._current_altitude
        
        # Simulate battery drain based on flight
        self._battery_percent -= 0.2
        
        print(f"[HAL] Flying to ({lat:.5f}, {lon:.5f}) at {altitude}m...")
        self._current_pos = position
        self._current_altitude = altitude
        time.sleep(2) # Simulate flight time

    def hover(self):
        """Command the drone to stop all horizontal movement."""
        print("[HAL] Drone HOVERING.")
        time.sleep(1)

    def return_to_launch(self):
        print("[HAL] Returning to home position...")
        self.goto(self._home_pos, altitude=30)
        self.land()

    # --- Actuator Commands ---
    
    def drop_payload(self):
        print("[HAL] 📦 PAYLOAD DROPPED.")
        time.sleep(1)

    def set_strobe(self, state: bool):
        if state:
            print("[HAL] 🚨 STROBE LIGHT ON.")
        else:
            print("[HAL] 💡 STROBE LIGHT OFF.")

    # --- Telemetry Getters ---
    
    def get_position(self) -> Tuple[float, float]:
        """Returns (lat, lon)"""
        return self._current_pos

    @property
    def altitude(self) -> float:
        """Returns altitude in meters"""
        # Simulate slight altitude drift
        self._current_altitude += random.uniform(-0.1, 0.1)
        return self._current_altitude

    def get_relative_vertical_velocity(self) -> float:
        """Used for motion-timed landing."""
        return random.uniform(-0.5, 0.5)

    def is_manual_control_active(self) -> bool:
        """
        Checks if the low-latency RC joystick is being used.
        """
        return False

    def get_telemetry(self) -> Dict[str, Any]:
        """
        [NEW] Provides a full state snapshot for the Safety Monitor.
        """
        # Simulate battery drain over time
        if self._is_armed:
            self._battery_percent -= 0.05 
            
        return {
            "is_armed": self._is_armed,
            "battery_percent": self._battery_percent,
            "altitude_m": self._current_altitude,
            "position": self._current_pos,
            "is_manual_control": self.is_manual_control_active()
        }