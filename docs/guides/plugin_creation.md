"""
Developer Guide: Adding New Plugins to COBALT

This guide shows how to create and register new capability plugins
that work with the BehaviorFactory (Bug #6 fix).
"""

# ====================================================================================
# Step 1: Create the Plugin Implementation
# ====================================================================================

# Example: Adding a Visual Detector Plugin

# File: drone/core/g3_capability_plugins/detectors/visual_detector.py

"""
Component 6: Detector (Visual Implementation)
A concrete implementation of BaseDetector using computer vision.
"""

import asyncio
import time
from typing import Optional
from .base import BaseDetector, Detection, Camera

class VisualDetector(BaseDetector):
    """
    Detects targets using computer vision (YOLOv8, etc.).
    """
    
    def __init__(self, camera: Camera, config: dict = None):
        super().__init__(camera)
        
        # Load configuration
        config = config or {}
        self.confidence_threshold = config.get("confidence_threshold", 0.85)
        self.model_path = config.get("model_path", "models/yolov8.pt")
        self.target_classes = config.get("target_classes", ["person"])
        
        # Initialize model (example)
        # from ultralytics import YOLO
        # self.model = YOLO(self.model_path)
        
        print(f"[VisualDetector] Initialized with threshold {self.confidence_threshold}")
        print(f"[VisualDetector] Model: {self.model_path}")
        print(f"[VisualDetector] Target classes: {self.target_classes}")

    async def detect(self) -> Optional[Detection]:
        """
        Runs computer vision detection on the camera frame.
        """
        # 1. Get frame
        frame = await self.camera.get_frame()
        
        # 2. Run inference (simulated here)
        await asyncio.sleep(0.1)  # Simulate inference time
        
        # Real implementation:
        # results = self.model(frame)
        # detections = results[0].boxes
        # 
        # for detection in detections:
        #     if detection.cls in self.target_classes:
        #         if detection.conf > self.confidence_threshold:
        #             return Detection(
        #                 timestamp=time.monotonic(),
        #                 class_label=detection.cls,
        #                 confidence=detection.conf,
        #                 position=(detection.x, detection.y)
        #             )
        
        # Simulated detection for now
        import random
        if random.random() > 0.95:
            return Detection(
                timestamp=time.monotonic(),
                class_label="person",
                confidence=0.92,
                position=(random.randint(100, 500), random.randint(100, 500))
            )
        
        return None


# ====================================================================================
# Step 2: Register in Factory Function
# ====================================================================================

# File: drone/core/g3_capability_plugins/__init__.py

# Add import at top:
from .detectors.visual_detector import VisualDetector

# Add to get_detector function:
def get_detector(detector_type: str, camera: Camera, config: Optional[Dict[str, Any]] = None) -> BaseDetector:
    config = config or {}
    
    if detector_type == "thermal_detector":
        threshold = config.get("threshold", 0.9)
        return ThermalDetector(camera, threshold=threshold)
    
    # NEW: Add visual detector
    elif detector_type == "visual_detector":
        return VisualDetector(camera, config)
    
    # Add more detector types here
    
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Available types: thermal_detector, visual_detector"  # UPDATE THIS!
        )


# ====================================================================================
# Step 3: Add Configuration Template
# ====================================================================================

# File: config/mission_config.yaml

# Add to detectors section:
detectors:
  thermal_detector:
    threshold: 0.9
    
  visual_detector:  # NEW
    model_path: "models/yolov8_rescue.pt"
    confidence_threshold: 0.85
    nms_threshold: 0.45
    target_classes:
      - "person"
      - "lifejacket"
      - "flotation_device"


# ====================================================================================
# Step 4: Update Mission JSON to Use New Plugin
# ====================================================================================

# File: missions/visual_search_mission.json

{
  "mission_id": "visual_search_001",
  "start_phase": "searching",
  "phases": {
    "searching": {
      "tasks": {
        "scout": {
          "action": "EXECUTE_SEARCH",
          "detector": "visual_detector",  # Use new detector!
          "strategy": "lawnmower"
        }
      },
      "transitions": {
        "on_event:target_found": {"scout": "goto:confirming"}
      }
    }
  }
}


# ====================================================================================
# Step 5: Test the New Plugin
# ====================================================================================

# File: tests/test_visual_detector.py

import asyncio
import pytest
from drone.core.g3_capability_plugins import get_detector
from drone.core.g4_platform_interface.sensors.cameras.base import SimulatedCamera

@pytest.mark.asyncio
async def test_visual_detector_loading():
    """Test that visual detector can be loaded via factory."""
    camera = SimulatedCamera(0)
    
    config = {
        "confidence_threshold": 0.9,
        "model_path": "models/test.pt"
    }
    
    detector = get_detector("visual_detector", camera, config)
    
    assert detector is not None
    assert detector.confidence_threshold == 0.9
    assert detector.model_path == "models/test.pt"

@pytest.mark.asyncio
async def test_visual_detector_detection():
    """Test that visual detector can perform detections."""
    camera = SimulatedCamera(0)
    detector = get_detector("visual_detector", camera)
    
    # Run detection
    detection = await detector.detect()
    
    # May or may not detect (simulated)
    if detection:
        assert detection.confidence > 0.0
        assert detection.class_label in ["person"]

@pytest.mark.asyncio
async def test_visual_detector_in_behavior():
    """Test visual detector integration with SearchBehavior."""
    from drone.core.g2_execution_core.behaviors import BehaviorFactory
    from drone.core.g1_mission_definition.phase import Task
    from drone.core.g2_execution_core.mission_state import MissionState
    from drone.core.g4_platform_interface.hal_simulated import SimulatedController
    
    # Setup
    config = {
        "detectors": {
            "visual_detector": {
                "confidence_threshold": 0.85
            }
        },
        "strategies": {
            "lawnmower": {
                "width": 100,
                "height": 100,
                "step": 20,
                "altitude": -30
            }
        }
    }
    
    hal = SimulatedController(config)
    await hal.connect()
    
    factory = BehaviorFactory(config)
    mission_state = MissionState()
    
    task = Task(
        action="EXECUTE_SEARCH",
        detector="visual_detector",
        strategy="lawnmower"
    )
    
    # Create behavior - should load visual detector
    behavior = factory.create(task, mission_state, hal)
    
    assert behavior.detector is not None
    assert behavior.detector.__class__.__name__ == "VisualDetector"


# ====================================================================================
# Complete Example: Adding a New Strategy
# ====================================================================================

# File: drone/core/g3_capability_plugins/strategies/spiral.py

"""
Component 7: Strategy (Spiral Search Pattern)
"""

import asyncio
import math
from .base import BaseStrategy, Waypoint, VehicleState

class SpiralStrategy(BaseStrategy):
    """
    Generates waypoints in an expanding spiral pattern.
    """
    
    def __init__(self, vehicle_state: VehicleState, config: dict):
        super().__init__(vehicle_state, config)
        
        # Configuration
        self.center_x = config.get("center_x", 0.0)
        self.center_y = config.get("center_y", 0.0)
        self.start_radius = config.get("start_radius", 10.0)
        self.end_radius = config.get("end_radius", 200.0)
        self.radius_step = config.get("radius_step", 10.0)
        self.altitude = config.get("altitude", -50.0)
        self.points_per_revolution = config.get("points_per_revolution", 12)
        
        # State
        self.current_radius = self.start_radius
        self.current_angle = 0.0
        self.angle_step = 360.0 / self.points_per_revolution
        
        print(f"[SpiralStrategy] Initialized. Spiral from {self.start_radius}m to {self.end_radius}m")
    
    async def next_waypoint(self) -> Waypoint:
        """Calculate next point in spiral."""
        await asyncio.sleep(0.1)  # Simulate calculation
        
        # Convert polar to cartesian
        angle_rad = math.radians(self.current_angle)
        x = self.center_x + self.current_radius * math.cos(angle_rad)
        y = self.center_y + self.current_radius * math.sin(angle_rad)
        
        waypoint = Waypoint(x=x, y=y, z=self.altitude)
        
        # Update for next waypoint
        self.current_angle += self.angle_step
        
        # Complete revolution? Expand spiral
        if self.current_angle >= 360.0:
            self.current_angle = 0.0
            self.current_radius += self.radius_step
            
            # Reached end? Reset
            if self.current_radius > self.end_radius:
                print("[SpiralStrategy] Spiral complete. Restarting.")
                self.current_radius = self.start_radius
        
        return waypoint


# Register in __init__.py:
from .strategies.spiral import SpiralStrategy

def get_strategy(strategy_type: str, vehicle_state: VehicleState, config: Optional[Dict[str, Any]] = None) -> BaseStrategy:
    config = config or {}
    
    if strategy_type == "lawnmower":
        return LawnmowerStrategy(vehicle_state, config)
    
    elif strategy_type == "spiral":  # NEW
        return SpiralStrategy(vehicle_state, config)
    
    else:
        raise ValueError(f"Unknown strategy type: '{strategy_type}'. Available types: lawnmower, spiral")


# Add to config:
strategies:
  spiral:
    center_x: 0
    center_y: 0
    start_radius: 10
    end_radius: 200
    radius_step: 10
    altitude: -50
    points_per_revolution: 12


# ====================================================================================
# Plugin Development Checklist
# ====================================================================================

"""
[ ] 1. Implement plugin class extending appropriate base (BaseDetector, BaseStrategy, BaseActuator)
[ ] 2. Add __init__ that accepts (hardware/state, config)
[ ] 3. Implement required abstract methods
[ ] 4. Add import to __init__.py
[ ] 5. Add elif clause in factory function
[ ] 6. Update factory function error message with new type
[ ] 7. Add configuration template to mission_config.yaml
[ ] 8. Write unit tests
[ ] 9. Write integration test with BehaviorFactory
[ ] 10. Update documentation
[ ] 11. Test in real mission
"""


# ====================================================================================
# Best Practices
# ====================================================================================

"""
DO:
✓ Extend the appropriate base class
✓ Accept config dict in __init__
✓ Provide sensible defaults for all config values
✓ Log initialization parameters
✓ Handle errors gracefully
✓ Write comprehensive tests
✓ Document configuration options

DON'T:
✗ Hardcode configuration values
✗ Raise exceptions in __init__ (return None from methods instead)
✗ Depend on other plugins directly (use dependency injection)
✗ Modify global state
✗ Block the event loop (use async/await)
✗ Forget to update the factory function
"""


# ====================================================================================
# Common Patterns
# ====================================================================================

# Pattern 1: Configuration with Validation
class MyPlugin(BaseDetector):
    def __init__(self, camera, config=None):
        super().__init__(camera)
        config = config or {}
        
        # Required config with validation
        self.threshold = config.get("threshold", 0.9)
        if not 0.0 <= self.threshold <= 1.0:
            raise ValueError(f"threshold must be between 0 and 1, got {self.threshold}")
        
        # Optional config
        self.max_detections = config.get("max_detections", 10)
        
        # Derived config
        self.min_confidence = self.threshold * 0.8


# Pattern 2: Hardware Interface Usage
class MyPlugin(BaseDetector):
    async def detect(self):
        # Always use async for hardware interaction
        frame = await self.camera.get_frame()
        
        # Process frame
        result = self._process_frame(frame)
        
        return result


# Pattern 3: Graceful Degradation
class MyPlugin(BaseStrategy):
    async def next_waypoint(self):
        try:
            # Try to get current position
            current = await self.vehicle_state.get_position()
            waypoint = self._calculate_next(current)
        except Exception as e:
            # Fallback behavior
            print(f"[MyPlugin] Error: {e}. Using fallback.")
            waypoint = self._get_fallback_waypoint()
        
        return waypoint


# Pattern 4: State Management
class MyPlugin(BaseStrategy):
    def __init__(self, vehicle_state, config):
        super().__init__(vehicle_state, config)
        
        # Plugin-specific state
        self.waypoint_index = 0
        self.waypoints = self._generate_waypoints()
        self.completed = False
    
    async def next_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            self.completed = True
            self.waypoint_index = 0  # Restart
        
        waypoint = self.waypoints[self.waypoint_index]
        self.waypoint_index += 1
        return waypoint