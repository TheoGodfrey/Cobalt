"""
L4: Reusable asynchronous mission behaviors (The "Actor").
"""
import asyncio
import time
from .position import Position
from .drone import Drone, Telemetry
from .cameras.dual_camera import DualCameraSystem
# --- L3/L6/L8 Imports ---
from .detection.fusion_detector import FusionDetector # L6 Detector
from .detection.thermal_detector import ThermalDetector # L6 Detector
from .actuators import BaseActuator, get_actuator # L8 Actuator
from .config_models import Settings, PrecisionHoverConfig
# ------------------------
from typing import List, Tuple
from .cameras.base import Detection
from .navigation import CameraIntrinsicsHelper, image_to_world_position

class SearchBehavior:
    """L4: Encapsulates search behavior with pluggable Tools."""
    
    def __init__(self, 
                 drone: Drone, 
                 dual_camera: DualCameraSystem,
                 search_strategy,       # L7 Strategy (Legs)
                 flight_strategy,       # L7 Strategy (Legs)
                 config: Settings,
                 mqtt: "MqttClient", 
                 logger: "MissionLogger",
                 detector_name: str     # L6 Detector Tool Name (Eyes)
                 ):
        
        self.drone = drone
        self.dual_camera = dual_camera
        self.search_strategy = search_strategy
        self.flight_strategy = flight_strategy
        self.config = config
        self.mqtt = mqtt 
        self.logger = logger 
        self.iteration = 0
        
        # --- L6: Initialize the specific Detector Tool (Eyes) ---
        if detector_name == "fusion_detector":
             self.detector = FusionDetector(config.detection)
        elif detector_name == "thermal_threshold":
             self.detector = ThermalDetector(config.detection.thermal)
        else:
             self.detector = FusionDetector(config.detection) # Fallback

        self.intrinsics = CameraIntrinsicsHelper(config.cameras.visual.intrinsics)
        self.last_detections: List[Detection] = []
    
    async def search_step(self) -> Tuple[bool, Detection | None]:
        """
        Execute one asynchronous search step.
        Returns: (should_continue, confirmed_detection_or_none)
        """
        
        self.logger.log(f"Scanning at {self.drone.telemetry.position}...", "debug")
        
        # 1. Capture synchronized frame
        dual_frame = await self.dual_camera.capture_synchronized()
        
        # 2. Get latest telemetry (L10 Vehicle State)
        current_telemetry = self.drone.telemetry
        
        # 3. L6 Detect
        confirmed_detections = await self.detector.detect(dual_frame)
        self.last_detections = confirmed_detections
        
        self.iteration += 1
        
        # 4. Report raw detections to Coordinator's AI
        if confirmed_detections:
            best_detection = max(confirmed_detections, key=lambda d: d.confidence)
            
            best_detection.position_world = self._image_to_world_position(
                best_detection.position_image,
                current_telemetry
            )
            
            # Send an event to the Coordinator's Probabilistic AI
            await self.mqtt.publish(f"fleet/event/{self.drone.id}", {
                "type": "AI_DETECTION",
                "data": {
                    "position": best_detection.position_world.model_dump(),
                    "confidence": best_detection.confidence
                }
            })
            
            # 5. Check for confirmed target
            if best_detection.confidence > self.config.detection.fusion.fusion_threshold:
                 return True, best_detection

        # 6. Check for max iterations
        max_iter = self.config.mission.max_search_iterations
        if self.iteration >= max_iter:
            return False, None # Search complete (timeout)
        
        return True, None  
    
    def get_last_detections(self) -> List[Detection]:
        return self.last_detections

    def _image_to_world_position(self, 
                                 image_pos: tuple, 
                                 drone_telemetry: Telemetry) -> Position:
        """Wrapper for the real geolocation function."""
        return image_to_world_position(
            pixel=image_pos,
            drone_telemetry=drone_telemetry,
            intrinsics=self.intrinsics
        )

class DeliveryBehavior:
    """L4: Encapsulates payload delivery with pluggable Tools."""
    
    def __init__(self, 
                 drone: Drone, 
                 flight_strategy, 
                 config: PrecisionHoverConfig,
                 actuator_name: str,       # L8 Actuator Tool Name (Hands)
                 logger: "MissionLogger"
                 ):
        self.drone = drone
        self.flight_strategy = flight_strategy
        self.config = config 
        self.logger = logger
        
        # --- L8: Initialize the specific Actuator Tool (Hands) ---
        self.actuator_tool: BaseActuator = get_actuator(actuator_name)
    
    async def run(self, target_position: Position):
        """Run the full delivery phase."""
        
        # 1. Calculate approach position using L7 Strategy
        delivery_position = self.flight_strategy.get_next_position(self.drone, target_position)
        
        self.logger.log(f"Flying to delivery point: {delivery_position}", "info")
        await self.drone.go_to(delivery_position)
        
        # 2. L8 Perform the action using the configured Actuator Tool
        await self.actuator_tool.perform_action(self.drone, params={"duration_s": 2.0})
        
        self.logger.log("Delivery Actuator action complete.", "info")