"""
Plugin Factory Functions
Provides factory functions to instantiate capability plugins (G3 components).

These functions act as a simple plugin registry, mapping string identifiers
to concrete plugin implementations.

REFACTOR: This is the new "gatekeeper" that validates hardware.
"""

from typing import Any, Dict, Optional, List, Set # NEW: Added List, Set

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
# Maps plugin *name* (string) to a set of required hardware strings.
PLUGIN_HARDWARE_REQUIREMENTS: Dict[str, Set[str]] = {
    # Detectors
    "thermal_detector": {"camera_thermal", "gps"},

    # Strategies
    "lawnmower": {"gps"},
    "goto_target": {"gps"},
    "rth_strategy": {"gps"}, # Assuming RTH also needs GPS

    # Actuators
    "payload_dropper": {"dropper_mechanism"},
    
    # Add all other plugins here
    # "visual_detector": {"camera_rgb", "gpu"},
    # "lidar_detector": {"lidar"},
}


# NEW: Helper function to perform the validation
def _check_hardware_requirements(plugin_name: str, available_hardware: List[str]):
    """
    Checks if the drone's hardware meets the plugin's requirements.
    Raises an error if hardware is missing.
    """
    # Get the set of required hardware for this plugin
    required_hw = PLUGIN_HARDWARE_REQUIREMENTS.get(plugin_name)

    # If the plugin isn't in the map or has no requirements, it's valid
    if not required_hw:
        print(f"[PluginFactory] No hardware requirements listed for '{plugin_name}'. Allowing.")
        return

    available_hw_set = set(available_hardware)
    
    # Check if all required items are in the available set
    if not required_hw.issubset(available_hw_set):
        missing_hw = required_hw - available_hw_set
        raise EnvironmentError(
            f"Cannot load plugin '{plugin_name}'. "
            f"Missing required hardware: {list(missing_hw)}. "
            f"Available hardware: {available_hardware}"
        )
    
    # If we get here, all hardware is present
    print(f"[PluginFactory] Hardware check PASSED for '{plugin_name}'.")


# NEW: Factory functions now accept hardware_list
def get_detector(
    detector_type: str, 
    camera: Camera, 
    config: Optional[Dict[str, Any]] = None,
    hardware_list: List[str] = []  # NEW
) -> BaseDetector:
    """
    Factory function to create detector plugin instances.
    """
    # NEW: Validation step
    _check_hardware_requirements(detector_type, hardware_list)
    
    config = config or {}
    
    if detector_type == "thermal_detector":
        # FIX: Pass the entire config object, not threshold as a keyword argument.
        return ThermalDetector(camera, config)
    
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Available types: thermal_detector"
        )


# NEW: Factory functions now accept hardware_list
def get_strategy(
    strategy_type: str, 
    vehicle_state: VehicleState, 
    config: Optional[Dict[str, Any]] = None,
    hardware_list: List[str] = [] # NEW
) -> BaseStrategy:
    """
    Factory function to create strategy plugin instances.
    """
    # NEW: Validation step
    _check_hardware_requirements(strategy_type, hardware_list)

    config = config or {}
    
    if strategy_type == "lawnmower":
        return LawnmowerStrategy(vehicle_state, config)
    
    elif strategy_type == "goto_target":
        return GoToTargetStrategy(vehicle_state, config)
        
    # HACK: Add a simple RTH strategy that wasn't defined
    elif strategy_type == "rth_strategy":
        # Returns a fixed home point (0,0, -50)
        home_config = {"target_x": 0, "target_y": 0, "altitude": -50}
        return GoToTargetStrategy(vehicle_state, home_config)
    
    else:
        raise ValueError(
            f"Unknown strategy type: '{strategy_type}'. "
            f"Available types: lawnmower, goto_target, rth_strategy"
        )


# NEW: Factory functions now accept hardware_list
def get_actuator(
    actuator_type: str, 
    hardware: ActuatorHardware, 
    config: Optional[Dict[str, Any]] = None,
    hardware_list: List[str] = [] # NEW
) -> BaseActuator:
    """
    Factory function to create actuator plugin instances.
    """
    # NEW: Validation step
    _check_hardware_requirements(actuator_type, hardware_list)
    
    config = config or {}
    
    if actuator_type == "payload_dropper":
        return PayloadDropper(hardware, config)
    
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