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
# --- FIX: Import explicit position types ---
from ..utils.position import LocalPosition, GlobalPosition

# --- Re-use Waypoint from G3 as intended ---
from ..g3_capability_plugins.strategies.base import Waypoint


log = logging.getLogger(__name__)

# --- Constants ---
# Default connection string for drones broadcasting over Wi-Fi
DEFAULT_CONNECTION_STRING = 'udp:192.168.2.1:14550'
CONNECTION_TIMEOUT = 30  # seconds
ARM_TIMEOUT = 10 # seconds
TAKEOFF_TIMEOUT = 30 # seconds
GOTO_TIMEOUT = 60 # seconds per waypoint
ALTITUDE_REACHED_THRESHOLD = 0.95 # 95% of target altitude
ARRIVAL_DISTANCE_THRESHOLD = 1.0 # 1 meter

# --- NEW: Helper function for GOTO logic ---
def get_distance_metres(aLocation1: LocationGlobalRelative, aLocation2: LocationGlobalRelative) -> float:
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This is a standard DroneKit helper function.
    """
    if aLocation1 is None or aLocation2 is None:
        return 0.0
        
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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

    # --- THIS IS THE FIX ---
    def __init__(self, config: dict):
        """
        Initializes the ArduPilot controller.

        Args:
            config: A dictionary, typically from mission_config.yaml, containing
                    a 'connection_string' and 'camera_stream_url'.
        """
        # Call super() to initialize self.config and self.vehicle_state
        super().__init__(config)
        
        # self.config is now guaranteed to exist from the base class
        self.connection_string = self.config.get('connection_string', DEFAULT_CONNECTION_STRING)
        
        # Assumes the video stream URL is separate from MAVLink
        self.camera_stream_url = self.config.get('camera_stream_url', 'rtsp://192.168.2.1:554/stream')
    # --- END OF FIX ---
        
        self.vehicle: Optional[connect] = None
        self._camera = ArduPilotCamera(self.camera_stream_url)
        self._lidar = SimulatedLidar(0) # <-- NEW: Placeholder
        self._actuator = SimulatedActuator() # <-- NEW: Placeholder
        
        # Internal state to prevent spamming listeners
        self._last_mode = None
        self._last_armed = None
        
        # --- NEW: Home location for local coordinate translation ---
        self._home_location: Optional[LocationGlobalRelative] = None
        self._home_lat_cos: float = 1.0 # Cosine of home latitude
        # --- End of NEW ---
        
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
                await asyncio.sleep(0.1) # Use asyncio sleep

            # Arm the vehicle
            self.vehicle.armed = True
            start_time = time.time()
            while not self.vehicle.armed:
                if time.time() - start_time > ARM_TIMEOUT:
                    log.error("[HAL-ArduPilot] Failed to arm vehicle.")
                    return False
                await asyncio.sleep(0.1) # Use asyncio sleep
                
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
                await asyncio.sleep(0.1) # Use asyncio sleep
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
                await asyncio.sleep(0.5) # Use asyncio sleep
                
        except Exception as e:
            log.error(f"[HAL-ArduPilot] Error during takeoff: {e}")
            return False

    # --- THIS IS THE FIX ---
    async def goto(self, waypoint: Waypoint, groundspeed: float = 5.0) -> bool:
        """
        Commands the vehicle to fly to a new local waypoint.
        Converts local waypoint to global coordinates before sending.
        Blocks until arrival or failure.
        """
        if not self.vehicle or not self.vehicle.armed:
            log.warning(f"[HAL-ArduPilot] Cannot goto: Vehicle not armed.")
            return False
            
        if self.vehicle.mode.name != "GUIDED":
            log.warning(f"[HAL-ArduPilot] Cannot goto: Vehicle not in GUIDED mode.")
            return False
            
        if not self._home_location:
            log.error("[HAL-ArduPilot] Cannot goto: Home location not set. Unable to translate local waypoint.")
            return False

        try:
            # 1. Translate local Waypoint to global coordinates
            global_target = self._translate_local_to_global(waypoint)
            
            # 2. Create DroneKit location object
            # Note: ArduPilot alt is relative to home, which matches our Z logic
            target_location = LocationGlobalRelative(global_target.lat, global_target.lon, global_target.alt)
            
            log.info(f"[HAL-ArduPilot] GOTO Local: ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})m")
            log.info(f"[HAL-ArduPilot] GOTO Global: ({global_target.lat:.6f}, {global_target.lon:.6f}, {global_target.alt:.1f})m")

            # 3. Send command
            self.vehicle.simple_goto(target_location, groundspeed=groundspeed)
            
            # 4. Block until arrival
            start_time = time.time()
            
            while self.vehicle.mode.name == "GUIDED":
                if time.time() - start_time > GOTO_TIMEOUT:
                    log.error("[HAL-ArduPilot] GOTO command timed out.")
                    await self.stop_movement() # Loiter
                    return False
                    
                current_loc = self.vehicle.location.global_relative_frame
                remaining_distance = get_distance_metres(current_loc, target_location)
                
                if remaining_distance < ARRIVAL_DISTANCE_THRESHOLD:
                    log.info("[HAL-ArduPilot] GOTO: Arrived at waypoint.")
                    return True
                    
                await asyncio.sleep(0.5)
            
            log.warning("[HAL-ArduPilot] GOTO failed: Vehicle exited GUIDED mode.")
            return False

        except Exception as e:
            log.error(f"[HAL-ArduPilot] Failed to execute goto: {e}")
            return False
    # --- END OF FIX ---

    # --- THIS IS THE FIX ---
    async def set_target_waypoint(self, waypoint: Waypoint):
        """
        Sets a new target waypoint for the vehicle to fly towards.
        NON-BLOCKING.
        """
        if not self.vehicle or not self.vehicle.armed or self.vehicle.mode.name != "GUIDED":
            # Don't send if not in a valid state
            return
            
        if not self._home_location:
            # Can't translate, so can't send command
            if self._last_mode: # Avoid spamming
                log.warning("[HAL-ArduPilot] set_target_waypoint: No home location, cannot translate.")
            return

        try:
            # 1. Translate local Waypoint to global coordinates
            global_target = self._translate_local_to_global(waypoint)
            
            # 2. Create DroneKit location object
            target_location = LocationGlobalRelative(global_target.lat, global_target.lon, global_target.alt)

            # 3. Send command (non-blocking)
            # We assume a default groundspeed
            self.vehicle.simple_goto(target_location, groundspeed=5.0)
            
            # 4. Return immediately
            
        except Exception as e:
            # Log error but don't block
            log.error(f"[HAL-ArduPilot] Failed to execute set_target_waypoint: {e}")
    # --- END OF FIX ---

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
    
    # --- NEW: Coordinate Translation Helpers ---
    def _translate_global_to_local(self, global_pos: GlobalPosition) -> LocalPosition:
        """Converts global Lat/Lon/Alt to local X/Y/Z meters from home."""
        if not self._home_location:
            # No home set, cannot translate
            return LocalPosition(0, 0, 0)
            
        # Calculate X (East) and Y (North)
        # X = delta_lon, Y = delta_lat
        y = (global_pos.lat - self._home_location.lat) * 111111.0
        x = (global_pos.lon - self._home_location.lon) * 111111.0 * self._home_lat_cos
        
        # Z is altitude difference (Down is positive)
        # ArduPilot alt is relative to home, so this is correct.
        z = -(global_pos.alt - self._home_location.alt)
        
        return LocalPosition(x=x, y=y, z=z)

    def _translate_local_to_global(self, local_pos: Waypoint) -> GlobalPosition:
        """Converts local X/Y/Z meters Waypoint to global Lat/Lon/Alt."""
        if not self._home_location:
            raise ValueError("Home location is not set. Cannot translate to global coordinates.")
            
        # X is East, Y is North
        lat = self._home_location.lat + (local_pos.y / 111111.0)
        lon = self._home_location.lon + (local_pos.x / (111111.0 * self._home_lat_cos))
        
        # Z is Down (NED), Alt is Up
        # We assume relative altitude from home
        alt = self._home_location.alt + (-local_pos.z)
        
        # Ensure altitude is not below 0 (e.g. if home alt is 0)
        if alt < 0:
            alt = 0.0
            
        return GlobalPosition(lat=lat, lon=lon, alt=alt)
    # --- End of NEW ---
    
    def _update_full_telemetry(self):
        """Helper function to get all data and push a new Telemetry update."""
        if not self.vehicle:
            return

        # Get current state from VehicleState
        telem = self.vehicle_state.current

        # --- Get Location ---
        loc = self.vehicle.location.global_relative_frame # <-- Get the object
        
        # --- FIX: Check object and its attributes before use (Race Condition) ---
        if not (loc and loc.lat is not None and loc.lon is not None and loc.alt is not None):
            # We don't have a valid location, skip update
            return
        # --- End of FIX ---

        # --- NEW: Set home location on first valid fix ---
        if not self._home_location and loc.lat != 0 and loc.lon != 0:
            log.info(f"[HAL-ArduPilot] Setting Home (Origin) to: {loc.lat}, {loc.lon}")
            self._home_location = loc
            self._home_lat_cos = math.cos(math.radians(loc.lat))
        # --- End of NEW ---

        # --- FIX: Populate explicit GlobalPosition object ---
        global_pos = GlobalPosition(lat=loc.lat, lon=loc.lon, alt=loc.alt)
        # --- End of FIX ---
        
        # --- FIX: Populate explicit LocalPosition object ---
        local_pos = self._translate_global_to_local(global_pos)
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
            position_global=global_pos, # <-- NEW
            position_local=local_pos,   # <-- NEW
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