"""
Plugin Factory Functions
Provides factory functions to instantiate capability plugins (G3 components).

These functions act as a simple plugin registry, mapping string identifiers
to concrete plugin implementations.

REFACTOR: Added hardware-aware validation.
"""

from typing import Any, Dict, Optional, List, Set  # NEW: Added List, Set

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


# NEW: Source of truth for hardware requirements.
# Maps plugin *name* (str) to a set of required hardware strings.
PLUGIN_HARDWARE_REQUIREMENTS: Dict[str, Set[str]] = {
    # Detectors
    "thermal_detector": {"camera_thermal"},
    "visual_detector": {"camera_rgb"},

    # Strategies
    "lawnmower": {"gps"},
    "goto_target": {"gps"},
    "rth_strategy": {"gps"}, # Assuming a strategy named this exists

    # Actuators
    "payload_dropper": {"dropper_mechanism"},
    
    # Example of a plugin with no hardware requirements
    "logging_strategy": set(),
}


# NEW: Helper function to perform the validation
def _check_hardware_requirements(plugin_name: str, hardware_list: List[str]):
    """
    Checks if the drone's hardware meets the plugin's requirements.
    Raises an error if hardware is missing.
    """
    # Get the set of required hardware for this plugin
    required_hw = PLUGIN_HARDWARE_REQUIREMENTS.get(plugin_name)

    # If the plugin isn't in the map or has no requirements, it's valid
    if not required_hw:
        print(f"[PluginFactory] No hardware requirements found for '{plugin_name}'. Skipping check.")
        return

    available_hw_set = set(hardware_list)
    
    # Check if all required items are in the available set
    if not required_hw.issubset(available_hw_set):
        missing_hw = required_hw - available_hw_set
        raise EnvironmentError(  # Or a custom exception
            f"Cannot load plugin '{plugin_name}'. "
            f"Missing required hardware: {list(missing_hw)}. "
            f"Available hardware: {hardware_list}"
        )
    
    print(f"[PluginFactory] Hardware check PASSED for '{plugin_name}'.")


def get_detector(detector_type: str, 
                 camera: Camera, 
                 config: Optional[Dict[str, Any]] = None,
                 hardware_list: List[str] = None) -> BaseDetector: # NEW: Added hardware_list
    """
    Factory function to create detector plugin instances.
    
    Args:
        detector_type: The type of detector to create (e.g., "thermal_detector")
        camera: The camera hardware interface (provided by HAL)
        config: Optional configuration dictionary for the detector
        hardware_list: List of available hardware strings
        
    Returns:
        An instance of a BaseDetector subclass
        
    Raises:
        ValueError: If the detector_type is not recognized
        EnvironmentError: If required hardware is missing
        
    Example:
        >>> camera = hal.get_camera(0)
        >>> detector = get_detector("thermal_detector", camera, {"threshold": 0.95}, ["camera_thermal", "gps"])
    """
    config = config or {}
    hardware_list = hardware_list or []
    
    # NEW: Validation step
    _check_hardware_requirements(detector_type, hardware_list)
    
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


def get_strategy(strategy_type: str, 
                 vehicle_state: VehicleState, 
                 config: Optional[Dict[str, Any]] = None,
                 hardware_list: List[str] = None) -> BaseStrategy: # NEW: Added hardware_list
    """
    Factory function to create strategy plugin instances.
    
    Args:
        strategy_type: The type of strategy to create (e.g., "lawnmower")
        vehicle_state: The vehicle state object (provided by HAL)
        config: Optional configuration dictionary for the strategy
        hardware_list: List of available hardware strings
        
    Returns:
        An instance of a BaseStrategy subclass
        
    Raises:
        ValueError: If the strategy_type is not recognized
        EnvironmentError: If required hardware is missing
        
    Example:
        >>> strategy_config = { ... }
        >>> strategy = get_strategy("lawnmower", vehicle_state, strategy_config, ["gps"])
    """
    config = config or {}
    hardware_list = hardware_list or []

    # NEW: Validation step
    _check_hardware_requirements(strategy_type, hardware_list)
    
    if strategy_type == "lawnmower":
        return LawnmowerStrategy(vehicle_state, config)
    
    # --- ADDED THIS SECTION ---
    elif strategy_type == "goto_target":
        return GoToTargetStrategy(vehicle_state, config)
    # --- END OF ADDED SECTION ---

    # Add more strategy types here as they are implemented
    # elif strategy_type == "rth_strategy":
    #     return RthStrategy(vehicle_state, config)
    
    else:
        # --- UPDATED THE ERROR MESSAGE ---
        raise ValueError(
            f"Unknown strategy type: '{strategy_type}'. "
            f"Available types: lawnmower, goto_target"
        )


def get_actuator(actuator_type: str, 
                 hardware: ActuatorHardware, 
                 config: Optional[Dict[str, Any]] = None,
                 hardware_list: List[str] = None) -> BaseActuator: # NEW: Added hardware_list
    """
    Factory function to create actuator plugin instances.
    
    Args:
        actuator_type: The type of actuator to create (e.g., "payload_dropper")
        hardware: The actuator hardware interface (provided by HAL)
        config: Optional configuration dictionary for the actuator
        hardware_list: List of available hardware strings
        
    Returns:
        An instance of a BaseActuator subclass
        
    Raises:
        ValueError: If the actuator_type is not recognized
        EnvironmentError: If required hardware is missing
        
    Example:
        >>> hardware = hal.get_actuator_hardware(0)
        >>> actuator = get_actuator("payload_dropper", hardware, {"release_time_sec": 3}, ["dropper_mechanism"])
    """
    config = config or {}
    hardware_list = hardware_list or []

    # NEW: Validation step
    _check_hardware_requirements(actuator_type, hardware_list)
    
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