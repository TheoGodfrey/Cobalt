"""
Hardware Abstraction Layer (HAL) implementation for ArduPilot (MAVLink) devices.

This controller uses the 'dronekit' library to communicate with any MAVLink-compatible
flight controller, such as the one in the SplashDrone 4+.
"""

import time
import logging
from typing import Optional, Callable
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from .hal import BaseFlightController
from .vehicle_state import VehicleState, DroneStatus, FlightMode
from .sensors.cameras.base import BaseCamera
from pymavlink import mavutil

log = logging.getLogger(__name__)

# --- Constants ---
# Default connection string for drones broadcasting over Wi-Fi
DEFAULT_CONNECTION_STRING = 'udp:192.168.2.1:14550'
CONNECTION_TIMEOUT = 30  # seconds
ARM_TIMEOUT = 10 # seconds
TAKEOFF_TIMEOUT = 30 # seconds
ALTITUDE_REACHED_THRESHOLD = 0.95 # 95% of target altitude

class ArduPilotCamera(BaseCamera):
    """
    A virtual camera implementation for ArduPilot.
    It doesn't provide a video stream itself, but provides the *URL*
    to the stream which a G3 detector (like ThermalDetector) will open.
    """
    def __init__(self, stream_url: str):
        self._stream_url = stream_url
        log.info(f"[HAL-ArduPilot] Camera initialized. Stream URL: {self._stream_url}")

    def get_stream_url(self) -> str:
        """Returns the RTSP or HTTP stream URL for this camera."""
        return self._stream_url

    def start(self):
        # In this HAL, we don't control the stream, we just provide the URL.
        pass

    def stop(self):
        pass

    def capture(self):
        # Capture is handled by the G3 detector (e.g., OpenCV)
        log.warning("[HAL-ArduPilot] .capture() called on camera. This should be handled by a G3 detector.")
        return None

class ArduPilotController(BaseFlightController):
    """
    HAL implementation for ArduPilot using DroneKit.
    """

    def __init__(self, vehicle_state: VehicleState, config: dict):
        """
        Initializes the ArduPilot controller.

        Args:
            vehicle_state: The shared VehicleState object to update.
            config: A dictionary, typically from mission_config.yaml, containing
                    a 'connection_string' and 'camera_stream_url'.
        """
        # Note: This file's __init__ does not call super().__init__(config),
        # but it *does* match the base class signature (which Bug #7 fixed).
        # We manually assign vehicle_state and config.
        self.vehicle_state = vehicle_state
        self.config = config
        
        self.connection_string = config.get('connection_string', DEFAULT_CONNECTION_STRING)
        
        # Assumes the video stream URL is separate from MAVLink
        self.camera_stream_url = config.get('camera_stream_url', 'rtsp://192.168.2.1:554/stream')
        
        self.vehicle: Optional[connect] = None
        self._camera = ArduPilotCamera(self.camera_stream_url)
        
        # Internal state
        self._last_mode = None
        self._last_armed = None
        
        log.info(f"[HAL-ArduPilot] Initialized. Connecting to: {self.connection_string}")

    async def connect(self) -> bool:
        """
        Connects to the vehicle via MAVLink.
        """
        try:
            log.info(f"[HAL-ArduPilot] Attempting to connect to {self.connection_string}...")
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=CONNECTION_TIMEOUT)
            
            if not self.vehicle:
                log.error("[HAL-ArduPilot] Connection failed: Vehicle object is None.")
                return False

            log.info("[HAL-ArduPilot] Vehicle connected. Fetching parameters...")
            
            # --- Add Listeners to auto-update VehicleState ---
            # These listeners run in background threads managed by dronekit
            
            self.vehicle.add_attribute_listener('location.global_relative_frame', self._location_callback)
            self.vehicle.add_attribute_listener('attitude', self._attitude_callback)
            self.vehicle.add_attribute_listener('battery', self._battery_callback)
            self.vehicle.add_attribute_listener('mode', self._mode_callback)
            self.vehicle.add_attribute_listener('armed', self._armed_callback)
            self.vehicle.add_attribute_listener('system_status', self._status_callback)

            log.info("[HAL-ArduPilot] Listeners added. Connection successful.")
            self.vehicle_state.status = DroneStatus.ONLINE
            return True
            
        except APIException as e:
            log.error(f"[HAL-ArduPilot] Connection failed (APIException): {e}")
            return False
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Connection failed (Timeout or other error): {e}")
            return False

    async def disconnect(self):
        """
Deactivates listeners and closes the MAVLink connection.
"""
        log.info("[HAL-ArduPilot] Disconnecting...")
        if self.vehicle:
            # Remove all listeners
            self.vehicle.remove_attribute_listener('location.global_relative_frame', self._location_callback)
            self.vehicle.remove_attribute_listener('attitude', self._attitude_callback)
            self.vehicle.remove_attribute_listener('battery', self._battery_callback)
            self.vehicle.remove_attribute_listener('mode', self._mode_callback)
            self.vehicle.remove_attribute_listener('armed', self._armed_callback)
            self.vehicle.remove_attribute_listener('system_status', self._status_callback)
            
            self.vehicle.close()
            self.vehicle = None
            
        self.vehicle_state.status = DroneStatus.OFFLINE
        log.info("[HAL-ArduPilot] Disconnected.")

    async def arm(self) -> bool:
        """
        Arms the vehicle in GUIDED mode.
        """
        if not self.vehicle or not self.vehicle.is_armable:
            log.warning("[HAL-ArduPilot] Cannot arm: Vehicle not connected or not armable.")
            return False
            
        log.info("[HAL-ArduPilot] Arming vehicle...")
        try:
            # Set mode to GUIDED (required for API control)
            self.vehicle.mode = VehicleMode("GUIDED")
            start_time = time.time()
            while self.vehicle.mode.name != "GUIDED":
                if time.time() - start_time > ARM_TIMEOUT:
                    log.error("[HAL-ArduPilot] Failed to set GUIDED mode.")
                    return False
                time.sleep(0.1)

            # Arm the vehicle
            self.vehicle.armed = True
            start_time = time.time()
            while not self.vehicle.armed:
                if time.time() - start_time > ARM_TIMEOUT:
                    log.error("[HAL-ArduPilot] Failed to arm vehicle.")
                    return False
                time.sleep(0.1)
                
            log.info("[HAL-ArduPilot] Vehicle ARMED in GUIDED mode.")
            return True
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error during arming: {e}")
            return False

    async def disarm(self) -> bool:
        """
        Disarms the vehicle.
        """
        if not self.vehicle:
            return False
            
        log.info("[HAL-ArduPilot] Disarming vehicle...")
        try:
            self.vehicle.armed = False
            while self.vehicle.armed:
                time.sleep(0.1)
            log.info("[HAL-ArduPilot] Vehicle DISARMED.")
            return True
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error during disarming: {e}")
            return False

    async def takeoff(self, altitude_m: float) -> bool:
        """
        Commands the vehicle to take off to a specific altitude.
        """
        if not self.vehicle or not self.vehicle.armed:
            log.warning("[HAL-ArduPilot] Cannot takeoff: Vehicle not armed.")
            return False
            
        log.info(f"[HAL-ArduPilot] Taking off to {altitude_m} meters...")
        try:
            self.vehicle.simple_takeoff(altitude_m)
            
            # Wait until takeoff is complete
            start_time = time.time()
            while True:
                if time.time() - start_time > TAKEOFF_TIMEOUT:
                     log.error("[HAL-ArduPilot] Takeoff timed out.")
                     return False
                     
                alt = self.vehicle.location.global_relative_frame.alt
                if alt >= altitude_m * ALTITUDE_REACHED_THRESHOLD:
                    log.info(f"[HAL-ArduPilot] Takeoff complete. Reached {alt:.1f}m.")
                    return True
                time.sleep(0.5)
                
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error during takeoff: {e}")
            return False

    async def goto_location(self, lat: float, lon: float, alt_m: float, groundspeed: float = 5.0):
        """
        Commands the vehicle to fly to a new location.
        """
        if not self.vehicle or not self.vehicle.armed:
            log.warning(f"[HAL-ArduPilot] Cannot goto: Vehicle not armed.")
            return
            
        if self.vehicle.mode.name != "GUIDED":
            log.warning(f"[HAL-ArduPilot] Cannot goto: Vehicle not in GUIDED mode.")
            return
            
        target_location = LocationGlobalRelative(lat, lon, alt_m)
        self.vehicle.simple_goto(target_location, groundspeed=groundspeed)
        log.debug(f"[HAL-ArduPilot] Commanded goto: {lat}, {lon} @ {alt_m}m")

    async def set_roi(self, lat: float, lon: float, alt_m: float):
        """
        Commands the vehicle (and gimbal) to look at a Region of Interest.
        """
        if not self.vehicle:
            return
            
        try:
            location = LocationGlobalRelative(lat, lon, alt_m)
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_SET_ROI, # command
                0, # confirmation
                0, 0, 0, 0, # params 1-4
                lat, lon, alt_m
            )
            self.vehicle.send_mavlink(msg)
            log.info(f"[HAL-ArduPilot] Set ROI to: {lat}, {lon}, {alt_m}")
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Failed to set ROI: {e}")


    def get_telemetry(self) -> VehicleState:
        """
        Returns the most recent vehicle state.
        (Updated passively by listeners)
        """
        return self.vehicle_state

    # --- FIX for Bug #12 ---
    # Added camera_id parameter to match the base class signature
    def get_camera(self, camera_id: int) -> BaseCamera:
        """
        Returns an instance of the camera object.
        NOTE: This implementation ignores camera_id and always returns
        the primary camera.
        """
        # TODO: Extend to support multiple cameras if camera_id > 0
        if camera_id != 0:
            log.warning(f"[HAL-ArduPilot] Requested camera {camera_id}, but only camera 0 is supported. Returning camera 0.")
        return self._camera

    # --- Listener Callbacks (Private) ---
    # These methods are called by dronekit's background threads
    # and update the shared VehicleState object.

    def _location_callback(self, vehicle, attr_name, msg):
        if msg:
            self.vehicle_state.latitude = msg.lat
            self.vehicle_state.longitude = msg.lon
            self.vehicle_state.altitude_msl = msg.alt
            # DroneKit doesn't have a separate AGL, so we use relative_alt
            self.vehicle_state.relative_altitude = msg.alt 
            self.vehicle_state.vx = vehicle.velocity[0]
            self.vehicle_state.vy = vehicle.velocity[1]
            self.vehicle_state.vz = vehicle.velocity[2]
            
            # Simple check for GPS fix
            if vehicle.gps_0.fix_type >= 3: # 3 = 3D Fix
                self.vehicle_state.gps_fix = True
            else:
                self.vehicle_state.gps_fix = False

    def _attitude_callback(self, vehicle, attr_name, msg):
        if msg:
            self.vehicle_state.roll = msg.roll
            self.vehicle_state.pitch = msg.pitch
            self.vehicle_state.yaw = msg.yaw

    def _battery_callback(self, vehicle, attr_name, msg):
        if msg:
            self.vehicle_state.battery_percent = msg.level
            self.vehicle_state.battery_voltage = msg.voltage

    def _mode_callback(self, vehicle, attr_name, msg):
        if msg and self._last_mode != msg.name:
            mode_name = msg.name.upper()
            if mode_name == 'GUIDED':
                self.vehicle_state.flight_mode = FlightMode.GUIDED
            elif mode_name == 'MANUAL':
                self.vehicle_state.flight_mode = FlightMode.MANUAL
            elif mode_name == 'RTL':
                self.vehicle_state.flight_mode = FlightMode.RTH
            elif mode_name == 'LOITER':
                self.vehicle_state.flight_mode = FlightMode.HOLD
            else:
                self.vehicle_state.flight_mode = FlightMode.UNKNOWN
            self._last_mode = msg.name
            log.info(f"[HAL-ArduPilot] Mode changed to: {self.vehicle_state.flight_mode.name}")

    def _armed_callback(self, vehicle, attr_name, msg):
        if msg is not None and self._last_armed != msg:
            self.vehicle_state.armed = msg
            self._last_armed = msg
            log.info(f"[HAL-ArduPilot] Armed status: {msg}")

    def _status_callback(self, vehicle, attr_name, msg):
        if msg and msg.state != self.vehicle_state.status.name:
            # Map ArduPilot status to our internal DroneStatus
            status_name = msg.state.upper()
            if status_name == 'ACTIVE':
                self.vehicle_state.status = DroneStatus.ONLINE
            elif status_name == 'STANDBY':
                self.vehicle_state.status = DroneStatus.IDLE
            elif status_name == 'CRITICAL' or status_name == 'EMERGENCY':
                self.vehicle_state.status = DroneStatus.ERROR
            else:
                self.vehicle_state.status = DroneStatus.UNKNOWN