"""
Hardware Abstraction Layer (HAL) implementation for ArduPilot (MAVLink) devices.

This controller uses the 'dronekit' library to communicate with any MAVLink-compatible
flight controller, such as the one in the SplashDrone 4+.

--- THIS IS A PATCHED VERSION TO BE COMPATIBLE WITH vehicle_state.py ---
"""

import time
import logging
import math  # <-- FIX 1.1: Added math for radian conversion
import asyncio # <-- NEW: Added for async camera
import cv2     # <-- NEW: Added for OpenCV
from typing import Optional, Callable, Any, List # <-- NEW: Added Any & List
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from .hal import BaseFlightController
from .sensors.cameras.base import BaseCamera
from .sensors.lidar import Lidar, SimulatedLidar # <-- NEW
from ..g3_capability_plugins.actuators.base import ActuatorHardware # <-- NEW
from .hal_simulated import SimulatedActuator # <-- NEW
from pymavlink import mavutil

# --- FIX: Import the correct classes from vehicle_state.py ---
from .vehicle_state import VehicleState, Telemetry, VehicleModeEnum, GPSStatusEnum
# --- Re-use Waypoint from G3 as intended ---
from ..g3_capability_plugins.strategies.base import Waypoint


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
    A camera implementation for ArduPilot that uses OpenCV to capture
    a video stream from a URL.
    This fulfills the BaseCamera interface required by detectors.
    """
    def __init__(self, stream_url: str):
        """
        Initializes the camera.
        
        Args:
            stream_url: The RTSP or HTTP stream URL for this camera.
        """
        self._stream_url = stream_url
        self._cap: Optional[cv2.VideoCapture] = None
        self._is_streaming = False
        self._loop = asyncio.get_running_loop()
        log.info(f"[HAL-ArduPilot] Camera initialized. Stream URL: {self._stream_url}")

    async def start_streaming(self):
        """(Re)starts the video capture stream."""
        if self._is_streaming and self._cap:
            return # Already running
            
        log.info(f"[HAL-ArduPilot] Starting OpenCV capture from {self._stream_url}...")
        try:
            # VideoCapture is a blocking call, run in thread
            self._cap = await asyncio.to_thread(cv2.VideoCapture, self._stream_url)
            
            if not self._cap or not self._cap.isOpened():
                log.error(f"[HAL-ArduPilot] Failed to open video stream: {self._stream_url}")
                self._cap = None
                self._is_streaming = False
            else:
                self._is_streaming = True
                log.info("[HAL-ArduPilot] Video stream started.")
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error starting video stream: {e}")
            self._cap = None
            self._is_streaming = False

    async def stop_streaming(self):
        """Stops the video capture stream."""
        if self._cap:
            log.info("[HAL-ArduPilot] Stopping video stream...")
            # release() is blocking, run in thread
            await asyncio.to_thread(self._cap.release)
            self._cap = None
        self._is_streaming = False
        log.info("[HAL-ArduPilot] Video stream stopped.")

    async def get_frame(self) -> Any: # Returns a numpy array
        """
        Get the latest frame from the camera.
        
        This runs the blocking OpenCV read() call in a thread to
        avoid stalling the main asyncio event loop.
        """
        if not self._is_streaming or not self._cap:
            log.warning("[HAL-ArduPilot] get_frame called but stream is not active. Attempting to start...")
            await self.start_streaming()
            if not self._is_streaming:
                log.error("[HAL-ArduPilot] Stream failed to start. Cannot get frame.")
                return None # Failed to start
        
        try:
            # cap.read() is a blocking I/O operation
            ret, frame = await asyncio.to_thread(self._cap.read)
            
            if not ret:
                log.warning("[HAL-ArduPilot] Failed to read frame from stream. Stream may have ended. Reconnecting...")
                # Attempt to reconnect
                await self.stop_streaming()
                await self.start_streaming()
                return None
            
            return frame # This is the numpy array
            
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error reading frame: {e}")
            await self.stop_streaming() # Stop on error
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
        self._lidar = SimulatedLidar(0) # <-- NEW: Placeholder
        self._actuator = SimulatedActuator() # <-- NEW: Placeholder
        
        # Internal state to prevent spamming listeners
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
            self.vehicle.add_attribute_listener('gps_0', self._gps_callback) # Listen to GPS status

            log.info("[HAL-ArduPilot] Listeners added. Connection successful.")
            # Manually trigger an update to populate the state
            self._update_full_telemetry()
            return True
            
        except APIException as e:
            log.error(f"[HAL-ArduPilot] Connection failed (APIException): {e}")
            return False
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Connection failed (Timeout or other error): {e}")
            return False

    async def detect_hardware(self) -> List[str]:
        """
        Actively probes ArduPilot sensors.
        """
        detected_hardware = []
        log.info("[HAL-ArduPilot] Detecting hardware...")
        
        if not self.vehicle:
            log.error("[HAL-ArduPilot] Cannot detect hardware, vehicle not connected.")
            return []
            
        # 1. Check for GPS
        if self.vehicle.gps_0 and self.vehicle.gps_0.fix_type > 0:
            log.info("[HAL-ArduPilot] GPS detected (fix_type > 0).")
            detected_hardware.append("gps")
        else:
            log.warning("[HAL-ArduPilot] No GPS fix detected.")
            
        # 2. Check for Camera (by trying to get a frame)
        log.info("[HAL-ArduPilot] Probing camera stream...")
        try:
            # We just need to know if it *can* start
            await self._camera.start_streaming()
            
            if self._camera._is_streaming:
                log.info("[HAL-ArduPilot] Camera stream connected.")
                # We assume camera_0 is thermal as per config
                detected_hardware.append("camera_thermal")
                # We can stop streaming now, detectors will start it later
                await self._camera.stop_streaming()
            else:
                 log.warning("[HAL-ArduPilot] Failed to connect to camera stream.")
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Camera probe failed: {e}")

        # 3. Check for configured actuators (e.g., from params)
        # This is a proxy, as a generic dropper isn't "detectable"
        if "dropper_mechanism" in self.config.get('hardware', []):
             log.info("[HAL-ArduPilot] 'dropper_mechanism' is configured (assumed present).")
             detected_hardware.append("dropper_mechanism")
             
        # 4. Check for Lidar (proxy)
        if "lidar" in self.config.get('hardware', []):
             log.info("[HAL-ArduPilot] 'lidar' is configured (assumed present).")
             detected_hardware.append("lidar")

        return list(set(detected_hardware))

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
            self.vehicle.remove_attribute_listener('gps_0', self._gps_callback)
            
            self.vehicle.close()
            self.vehicle = None
            
        # Stop the camera stream
        await self._camera.stop_streaming()
            
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

    async def stop_movement(self):
        """Commands the vehicle to stop and loiter (BRAKE mode)."""
        if self.vehicle:
            log.warning("[HAL-ArduPilot] *** STOP MOVEMENT (BRAKE) ***")
            self.vehicle.mode = VehicleMode("BRAKE")

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

    def get_actuator_hardware(self, actuator_id: int) -> ActuatorHardware:
        """Returns an instance of the actuator hardware."""
        return self._actuator # Returning placeholder
        
    def get_lidar_sensor(self, sensor_id: int) -> Lidar:
        """Returns an instance of the Lidar sensor."""
        return self._lidar # Returning placeholder

    # --- Listener Callbacks (Private) ---
    # These methods are called by dronekit's background threads
    # and update the shared VehicleState object.
    
    # --- FIX: Refactored all callbacks to use the Telemetry object
    # and the vehicle_state.update() method.
    
    def _update_full_telemetry(self):
        """Helper function to get all data and push a new Telemetry update."""
        if not self.vehicle:
            return

        # Get current state from VehicleState
        telem = self.vehicle_state.current

        # --- Get Location ---
        pos = Waypoint(telem.position.x, telem.position.y, telem.position.z)
        loc = self.vehicle.location.global_relative_frame # <-- Get the object
        
        # --- FIX: Check object and its attributes before use (Race Condition) ---
        if loc and loc.lat is not None and loc.lon is not None and loc.alt is not None:
            # Note: G3 Strategies use X,Y,Z. This HAL provides Lat/Lon/Alt.
            # This is a different inconsistency, but for now we'll update
            # the Waypoint with Lat/Lon/Alt.
            pos = Waypoint(x=loc.lat, y=loc.lon, z=loc.alt, yaw=telem.position.yaw)
        # --- End of FIX ---
        
        # --- FIX 1.1: Get full attitude ---
        # Get defaults from current state
        att_roll = telem.attitude_roll
        att_pitch = telem.attitude_pitch
        att_yaw = telem.attitude_yaw
        
        # Overwrite with new data if available
        if self.vehicle.attitude:
            # Convert from radians (dronekit) to degrees (our spec)
            att_roll = math.degrees(self.vehicle.attitude.roll)
            att_pitch = math.degrees(self.vehicle.attitude.pitch)
            att_yaw = math.degrees(self.vehicle.attitude.yaw)
            pos.yaw = att_yaw # Also update yaw in Waypoint for consistency
        # --- End of FIX 1.1 ---

        # --- Get Battery ---
        batt = telem.battery_percent
        if self.vehicle.battery:
            batt = self.vehicle.battery.level

        # --- Get Mode ---
        mode = telem.mode
        if self.vehicle.mode:
            mode_name = self.vehicle.mode.name.upper()
            if mode_name == 'GUIDED':
                mode = VehicleModeEnum.GUIDED
            elif mode_name == 'MANUAL':
                mode = VehicleModeEnum.MANUAL
            elif mode_name == 'RTL':
                mode = VehicleModeEnum.RTL
            elif mode_name == 'LAND':
                mode = VehicleModeEnum.LAND
            elif mode_name == 'BRAKE': # <-- NEW
                mode = VehicleModeEnum.PAUSED # Map BRAKE to PAUSED
            elif self.vehicle.armed:
                mode = VehicleModeEnum.ARMED
            else:
                mode = VehicleModeEnum.DISARMED

        # --- Get GPS ---
        gps = telem.gps_status
        if self.vehicle.gps_0:
            fix = self.vehicle.gps_0.fix_type
            if fix >= 6:
                gps = GPSStatusEnum.RTK_FIXED
            elif fix == 5:
                gps = GPSStatusEnum.DGPS # Float RTK
            elif fix >= 3:
                gps = GPSStatusEnum.FIX_3D
            elif fix == 2:
                gps = GPSStatusEnum.FIX_2D
            else:
                gps = GPSStatusEnum.NO_FIX
        
        # --- Get Armed Status ---
        armed = telem.is_armed
        if self.vehicle.armed is not None:
            armed = self.vehicle.armed

        # --- Create and push new Telemetry object ---
        new_telemetry = Telemetry(
            timestamp=time.monotonic(),
            position=pos,
            mode=mode,
            gps_status=gps,
            battery_percent=batt,
            is_armed=armed,
            wind_speed=self.vehicle.wind_estimation.speed if self.vehicle.wind_estimation else 0,
            wind_direction=self.vehicle.wind_estimation.direction if self.vehicle.wind_estimation else 0,
            # --- FIX 1.1: Add new fields to constructor ---
            attitude_roll=att_roll,
            attitude_pitch=att_pitch,
            attitude_yaw=att_yaw
        )
        
        self.vehicle_state.update(new_telemetry)
        
    def _location_callback(self, vehicle, attr_name, msg):
        if msg:
            self._update_full_telemetry()

    def _attitude_callback(self, vehicle, attr_name, msg):
        if msg:
            self._update_full_telemetry()

    def _battery_callback(self, vehicle, attr_name, msg):
        if msg:
            self._update_full_telemetry()

    def _mode_callback(self, vehicle, attr_name, msg):
        if msg and self._last_mode != msg.name:
            self._last_mode = msg.name
            log.info(f"[HAL-ArduPilot] Mode changed to: {msg.name}")
            self._update_full_telemetry()

    def _armed_callback(self, vehicle, attr_name, msg):
        if msg is not None and self._last_armed != msg:
            self._last_armed = msg
            log.info(f"[HAL-ArduPilot] Armed status: {msg}")
            self._update_full_telemetry()

    def _gps_callback(self, vehicle, attr_name, msg):
        if msg:
            self._update_full_telemetry()