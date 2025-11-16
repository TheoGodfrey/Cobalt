# track1_shipping/mob_rescue.py
#
# [UPDATED]
# Imports are now corrected to use the 'track_1' package structure
# and the consolidated 'detectors.py' file.

import time
import random
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict, Any

# Import local package components
from .hal.dji_mavic3 import DJIMavic3
from .sensors.thermal_camera import ThermalCamera
from .ship_config import ShipConfig

# --- [IMPORTS UPDATED] ---
# All detection logic now comes from the single 'detectors.py' file
from .detection.detectors import (
    Detection,
    ClassicObjectDetector,
    EdgeObjectDetector,
    ThermalObjectDetector,
    SalienceFilter,
    CentroidTracker
)
# -----------------------------

# --- CHOOSE YOUR DETECTOR ---
# 1. YOLO (Heavy, Smart)
# detector_to_use = ThermalObjectDetector

# 2. Classic Threshold (Light, Absolute)
detector_to_use = ClassicObjectDetector

# 3. Classic Edge (Light, Relative)
# detector_to_use = EdgeObjectDetector
# -----------------------------


@dataclass
class SearchArea:
    """Simple dataclass for the search area."""
    lat_bounds: Tuple[float, float]
    lon_bounds: Tuple[float, float]

    def get_start_point(self):
        return (self.lat_bounds[0], self.lon_bounds[0]) # Bottom-left corner


class MOBRescue:
    """
    Simple, hardcoded MOB rescue mission.
    Supports multiple roles AND satellite command interruption.
    """

    def __init__(self, drone: DJIMavic3, thermal_camera: ThermalCamera, 
                 ship_config: ShipConfig, role: str = "deliver"):
        
        self.drone = drone
        self.camera = thermal_camera
        self.ship = ship_config
        self.role = role
        if self.role not in ["deliver", "strobe"]:
            raise ValueError(f"Unknown role: {self.role}")
            
        print(f"[Mission] Initialized with role: {self.role}")

        # --- Detection Pipeline ---
        self.detector = detector_to_use()
        self.salience = SalienceFilter(mission_type="mob")
        self.tracker = CentroidTracker(max_missed=10) # 10 frames = ~5s

        # --- Mission Constants ---
        self.SEARCH_ALTITUDE = 50       
        self.CONFIRMATION_HITS = 5 # Must see target 5 times to confirm
        
        # --- Mission State ---
        self.mission_status = "PENDING" # PENDING -> RUNNING -> (SUCCESS/ABORT_RTH/PAUSED)
        self.mission_result = {"status": "pending"}


    def execute(self, search_area: SearchArea):
        """Execute the full, linear MOB rescue mission."""
        try:
            self.mission_status = "RUNNING"
            print(f"🚁 MOB Rescue Mission Starting (Role: {self.role})")

            # Phase 1: Launch
            if self.mission_status == "RUNNING":
                self._launch()

            # Phase 2: Transit
            if self.mission_status == "RUNNING":
                self._transit_to_search(search_area)

            # Phase 3: Search (this is interruptible)
            target_gps = None
            if self.mission_status == "RUNNING":
                target_gps = self._search(search_area)

            # --- Check for Interruption ---
            if self.mission_status == "ABORT_RTH":
                print("[Mission] Aborted by operator command.")
                self.mission_result = {"status": "aborted_by_operator"}
            elif self.mission_status == "REDIRECT":
                print("[Mission] Redirected by operator command.")
                self.mission_result = {"status": "redirected_by_operator"}
                return self.mission_result # Don't RTH
            elif self.mission_status == "PAUSED":
                print("[Mission] Paused. Awaiting resume...")
                self._pause_loop()
                self.mission_result = {"status": "resumed_then_completed"}

            # --- Role-Based Action ---
            elif target_gps:
                print("[Mission] Target found.")
                if self.role == "deliver":
                    self._deliver_payload(target_gps)
                    self.mission_result = {"status": "success", "target": target_gps}
                elif self.role == "strobe":
                    self._signal_target(target_gps)
                    self.mission_result = {"status": "success", "target": target_gps}
            else:
                self.mission_result = {"status": "no_target"}

            # Phase 5: Return home (unless redirected)
            if self.mission_status != "REDIRECT":
                self._return_home()

            print(f"✅ Mission Complete: {self.mission_result['status']}")
            return self.mission_result

        except Exception as e:
            print(f"❌ MISSION FAILED: {e}")
            self.mission_result = {"status": "failure", "error": str(e)}
            try:
                self._return_home()
            except Exception as e_rth:
                print(f"❌ RTH FAILED: {e_rth}")
            return self.mission_result
            
    # --- Interruption Methods (called by RemoteControlWrapper) ---
    
    def abort_mission(self):
        if self.mission_status == "RUNNING":
            print("[Mission] ABORT command received!")
            self.mission_status = "ABORT_RTH"
            self.drone.hover() 

    def abort_and_redirect(self, position, altitude):
        if self.mission_status == "RUNNING":
            print(f"[Mission] REDIRECT command received. Flying to {position}@{altitude}m")
            self.mission_status = "REDIRECT"
            self.drone.goto(position, altitude=altitude)
            
    def pause_mission(self):
        if self.mission_status == "RUNNING":
            print("[Mission] PAUSE command received. Hovering.")
            self.mission_status = "PAUSED"
            self.drone.hover()
            
    def resume_mission(self):
        if self.mission_status == "PAUSED":
            print("[Mission] RESUME command received.")
            self.mission_status = "RUNNING" 
            
    def _pause_loop(self):
        """A blocking loop to wait while paused."""
        while self.mission_status == "PAUSED":
            print("...Paused. Hovering.")
            time.sleep(5)
        print("...Resuming mission (proceeding to RTH).")

    # --- Standard Mission Phases ---

    def _launch(self):
        print("Phase 1: Launching...")
        self.drone.arm()
        self.drone.takeoff()
        self.drone.climb_to(self.SEARCH_ALTITUDE)

    def _transit_to_search(self, search_area: SearchArea):
        start_point = search_area.get_start_point()
        print(f"Phase 2: Transiting to search area: {start_point}")
        self.drone.goto(start_point, altitude=self.SEARCH_ALTITUDE)

    def _search(self, search_area: SearchArea):
        print("Phase 3: Starting search pattern...")
        waypoints = self._generate_lawnmower(search_area)
        
        if not waypoints:
            print("⚠️  No waypoints generated for search.")
            return None

        for i, waypoint in enumerate(waypoints):
            if self.mission_status != "RUNNING":
                print("...Search loop aborted.")
                return None
        
            print(f"  Searching leg {i+1}/{len(waypoints)} to {waypoint}")
            self.drone.goto(waypoint, altitude=self.SEARCH_ALTITUDE)

            # (This loop simulates flight time to the waypoint)
            for _ in range(5): 
                if self.mission_status != "RUNNING":
                    print("...Search leg aborted (Satellite Command).")
                    return None
                
                if self.drone.is_manual_control_active():
                    print("...Search leg aborted (Pilot took manual control).")
                    self.pause_mission()
                    return None
            
                frame = self.camera.get_frame()
                if frame is None: time.sleep(0.5); continue

                detections = self.detector.detect(frame)
                context = {"altitude": self.drone.altitude}
                salient_results = self.salience.filter(detections, context)
                
                # Only update tracker with high-confidence blobs
                salient_detections = []
                if salient_results:
                    # Get detections that are "good enough" (score > 0.6)
                    salient_detections = [det for det, score in salient_results if score > 0.6]

                live_tracks = self.tracker.update(salient_detections)

                for track_id, track in live_tracks.items():
                    if track["hits"] >= self.CONFIRMATION_HITS:
                        print(f"🎯 TARGET CONFIRMED (Track ID: {track_id})")
                        confirmed_detection = track["last_detection"]
                        position = self._detection_to_gps(confirmed_detection)
                        return position
                
                time.sleep(0.5) 

        print("⚠️  Search complete. No target found.")
        return None

    def _generate_lawnmower(self, search_area: SearchArea):
        print(f"[STUB] Generating lawnmower for {self.ship.search_radius}m radius.")
        (min_lat, max_lat) = search_area.lat_bounds
        (min_lon, max_lon) = search_area.lon_bounds
        return [
            (min_lat, max_lon), (max_lat, max_lon),
            (max_lat, min_lon), (min_lat, min_lon)
        ]

    def _deliver_payload(self, target_position):
        print(f"Phase 4 (Deliver): Flying to target {target_position}")
        safe_drop_altitude = 20 
        self.drone.goto(target_position, altitude=safe_drop_altitude)
        self.drone.hover()
        print("...Hovering above target. Releasing payload.")
        self.drone.drop_payload()

    def _signal_target(self, target_position):
        print(f"Phase 4 (Strobe): Flying to target {target_position}")
        signal_altitude_high = 50
        signal_altitude_low = 30
        signal_cycles = 10        
        
        self.drone.goto(target_position, altitude=signal_altitude_high)
        self.drone.hover()
        self.drone.set_strobe(True)
        print(f"...Strobe on. Oscillating {signal_cycles} times.")
        
        for i in range(signal_cycles):
            if self.mission_status != "RUNNING":
                print("...Signaling aborted (Satellite Command).")
                break
            
            if self.drone.is_manual_control_active():
                print("...Signaling aborted (Pilot took manual control).")
                self.pause_mission()
                break
                
            print(f"  ...Cycle {i+1}: Going LOW")
            self.drone.goto(target_position, altitude=signal_altitude_low)
            time.sleep(2)
            
            if self.mission_status != "RUNNING":
                print("...Signaling aborted (Satellite Command).")
                break
            
            if self.drone.is_manual_control_active():
                print("...Signaling aborted (Pilot took manual control).")
                self.pause_mission()
                break
            
            print(f"  ...Cycle {i+1}: Going HIGH")
            self.drone.goto(target_position, altitude=signal_altitude_high)
            time.sleep(2) 
            
        print("...Signaling complete.")
        self.drone.set_strobe(False)

    def _return_home(self):
        print(f"Phase 5: Returning to home (motion comp: {self.ship.motion_compensation})")
        self.drone.goto(self.drone.home_pos, altitude=self.ship.return_altitude)

        if self.ship.motion_compensation == "aggressive":
            print("  ...performing aggressive motion-timed landing.")
            self._land_with_motion_tracking()
        else:
            print("  ...performing simple landing.")
            self.drone.land()
        
        self.drone.disarm()

    def _land_with_motion_tracking(self):
        print("  [STUB] Waiting for heave trough (low point)...")
        while True:
            heave_rate = self.drone.get_relative_vertical_velocity()
            if abs(heave_rate) < 0.1: 
                print("  [STUB] Heave stable. Landing fast.")
                break
            time.sleep(0.1)
        self.drone.land(descent_rate=2.0) 

    def _detection_to_gps(self, detection: Detection):
        """
        Converts pixel coordinates to GPS coordinates.
        Uses simple "gimbal-down" math.
        """
        print("  [Localizer] Converting pixel to GPS using 'gimbal-down' math...")
        (drone_lat, drone_lon) = self.drone.get_position()
        altitude = self.drone.altitude
        
        # Get camera specs (hardcoded for MVP)
        image_width_px = 640
        horizontal_fov_deg = 60 
        
        pixel_x, pixel_y = detection.centroid
        
        # Offset from center (in pixels)
        x_offset_px = pixel_x - (image_width_px / 2)
        y_offset_px = pixel_y - (image_width_px / 2) # (using width for both)
        
        # How many degrees per pixel?
        deg_per_px_h = horizontal_fov_deg / image_width_px
        
        # Calculate angle offset
        angle_offset_x = x_offset_px * deg_per_px_h
        angle_offset_y = y_offset_px * deg_per_px_h # (Assuming square pixels)

        # Use trigonometry to find ground offset (meters)
        ground_offset_x = altitude * np.tan(np.radians(angle_offset_x))
        ground_offset_y = altitude * np.tan(np.radians(angle_offset_y))

        # Convert ground offset (meters) to GPS offset (degrees)
        lat_offset_deg = ground_offset_y / 111111.0 # ~111km per degree lat
        lon_offset_deg = ground_offset_x / (111111.0 * np.cos(np.radians(drone_lat)))

        target_gps = (drone_lat + lat_offset_deg, drone_lon + lon_offset_deg)
        return target_gps