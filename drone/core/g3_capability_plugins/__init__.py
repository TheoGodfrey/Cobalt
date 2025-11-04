"""
Plugin Factory Functions
Provides factory functions to instantiate capability plugins (G3 components).

These functions act as a simple plugin registry, mapping string identifiers
to concrete plugin implementations.
"""

from typing import Any, Dict, Optional

# Import concrete detector implementations
from .detectors.thermal_detector import ThermalDetector
from .detectors.base import BaseDetector, Camera

# Import concrete strategy implementations
from .strategies.lawnmower import LawnmowerStrategy
from .strategies.base import BaseStrategy, VehicleState
# --- ADDED THIS IMPORT ---
from .strategies.goto_target_strategy import GoToTargetStrategy

# Import concrete actuator implementations
from .actuators.payload_dropper import PayloadDropper
from .actuators.base import BaseActuator, ActuatorHardware


def get_detector(detector_type: str, camera: Camera, config: Optional[Dict[str, Any]] = None) -> BaseDetector:
    """
    Factory function to create detector plugin instances.
    
    Args:
        detector_type: The type of detector to create (e.g., "thermal_detector")
        camera: The camera hardware interface (provided by HAL)
        config: Optional configuration dictionary for the detector
        
    Returns:
        An instance of a BaseDetector subclass
        
    Raises:
        ValueError: If the detector_type is not recognized
        
    Example:
        >>> camera = hal.get_camera(0)
        >>> detector = get_detector("thermal_detector", camera, {"threshold": 0.95})
    """
    config = config or {}
    
    if detector_type == "thermal_detector":
        threshold = config.get("threshold", 0.9)
        return ThermalDetector(camera, threshold=threshold)
    
    # Add more detector types here as they are implemented
    # elif detector_type == "visual_detector":
    #     return VisualDetector(camera, config)
    # elif detector_type == "fusion_detector":
    #     return FusionDetector(camera, config)
    
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Available types: thermal_detector"
        )


def get_strategy(strategy_type: str, vehicle_state: VehicleState, config: Optional[Dict[str, Any]] = None) -> BaseStrategy:
    """
    Factory function to create strategy plugin instances.
    
    Args:
        strategy_type: The type of strategy to create (e.g., "lawnmower")
        vehicle_state: The vehicle state object (provided by HAL)
        config: Optional configuration dictionary for the strategy
        
    Returns:
        An instance of a BaseStrategy subclass
        
    Raises:
        ValueError: If the strategy_type is not recognized
        
    Example:
        >>> strategy_config = {
        ...     "origin_x": 0, "origin_y": 0,
        ...     "width": 1000, "height": 1000,
        ...     "step": 50, "altitude": -50
        ... }
        >>> strategy = get_strategy("lawnmower", vehicle_state, strategy_config)
    """
    config = config or {}
    
    if strategy_type == "lawnmower":
        return LawnmowerStrategy(vehicle_state, config)
    
    # --- ADDED THIS SECTION ---
    elif strategy_type == "goto_target":
        return GoToTargetStrategy(vehicle_state, config)
    # --- END OF ADDED SECTION ---

    # Add more strategy types here as they are implemented
    # elif strategy_type == "orbit":
    #     return OrbitStrategy(vehicle_state, config)
    
    else:
        # --- UPDATED THE ERROR MESSAGE ---
        raise ValueError(
            f"Unknown strategy type: '{strategy_type}'. "
            f"Available types: lawnmower, goto_target"
        )


def get_actuator(actuator_type: str, hardware: ActuatorHardware, config: Optional[Dict[str, Any]] = None) -> BaseActuator:
    """
    Factory function to create actuator plugin instances.
    
    Args:
        actuator_type: The type of actuator to create (e.g., "payload_dropper")
        hardware: The actuator hardware interface (provided by HAL)
        config: Optional configuration dictionary for the actuator
        
    Returns:
        An instance of a BaseActuator subclass
        
    Raises:
        ValueError: If the actuator_type is not recognized
        
    Example:
        >>> hardware = hal.get_actuator_hardware(0)
        >>> actuator = get_actuator("payload_dropper", hardware, {"release_time_sec": 3})
    """
    config = config or {}
    
    if actuator_type == "payload_dropper":
        return PayloadDropper(hardware, config)
    
    # Add more actuator types here as they are implemented
    # elif actuator_type == "winch":
    #     return WinchActuator(hardware, config)
    
    else:
        raise ValueError(
            f"Unknown actuator type: '{actuator_type}'. "
            f"Available types: payload_dropper"
        )


# Export the factory functions and base classes
__all__ = [
    'get_detector',
    'get_strategy', 
    'get_actuator',
    'BaseDetector',
    'BaseStrategy',
    'BaseActuator'
]
