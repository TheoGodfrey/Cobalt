"""
Component 9: Hardware Abstraction Layer (HAL)
Unified abstract interface to vehicle hardware.

This is the ONLY component that should talk to hardware (or the simulator).
It provides a high-level API for other components (like Behaviors) to use.

It owns and manages:
- The VehicleState (Component 10)
- The low-level hardware interfaces (e.g., MAVLink connection)
- The sensor objects (e.g., BaseCamera)
- The actuator objects (e.g., ActuatorHardware)
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any

# Import component 10
from .vehicle_state import VehicleState

# Import protocols (interfaces) from G3/G4
from ..g3_capability_plugins.strategies.base import Waypoint
from .sensors.cameras.base import BaseCamera
from ..g3_capability_plugins.actuators.base import ActuatorHardware

# --- HAL Base Class ---

class BaseFlightController(ABC):
    """
    The abstract interface for the HAL.
    All high-level components (G2, G3) code against this interface.
    """
    
    # --- FIX for Bug #7 ---
    # Added config parameter to match the derived class's super() call
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initializes the HAL.
        
        Args:
            config: An optional configuration dictionary.
        """
        self.config = config if config is not None else {}
        self.vehicle_state = VehicleState()

    @abstractmethod
    async def connect(self):
        """Establish connection to the flight controller."""
        pass
        
    @abstractmethod
    async def arm(self):
        """Arm the vehicle."""
        pass
        
    @abstractmethod
    async def disarm(self):
        """Disarm the vehicle."""
        pass
        
    @abstractmethod
    async def goto(self, waypoint: Waypoint) -> bool:
        """
        Fly to a specific waypoint.
        Returns True on arrival, False on failure.
        """
        pass
        
    @abstractmethod
    async def land(self) -> bool:
        """Land at the current position."""
        pass
        
    @abstractmethod
    def get_camera(self, camera_id: int) -> BaseCamera:
        """Get an interface to a camera."""
        pass
        
    @abstractmethod
    def get_actuator_hardware(self, actuator_id: int) -> ActuatorHardware:
        """Get an interface to an actuator."""
        pass


# --- Factory Function (FIX for Bug #3) ---

def get_flight_controller(config: Dict[str, Any], drone_id: str) -> BaseFlightController:
    """
    Factory function to instantiate the appropriate HAL controller.
    
    This function looks up the drone's configuration in the fleet config,
    determines which HAL implementation to use, and instantiates it with
    the appropriate configuration.
    
    Args:
        config: The complete mission configuration dictionary (from mission_config.yaml)
        drone_id: The unique identifier for this drone (e.g., "scout_1", "payload_1")
        
    Returns:
        An instance of a BaseFlightController subclass
        
    Raises:
        ValueError: If the drone_id is not found in the fleet configuration
        ValueError: If the HAL type is not recognized
        KeyError: If required configuration keys are missing
        
    Example:
        >>> config = load_config('config/mission_config.yaml')
        >>> hal = get_flight_controller(config, "scout_1")
        >>> await hal.connect()
    """
    
    # Import concrete implementations here to avoid circular imports
    from .hal_simulated import SimulatedController
    from .hal_ardupilot import ArduPilotController
    from .hal_dji import DJIController
    from .hal_omniverse import OmniverseController
    
    # Validate that we have a fleet configuration
    if 'fleet' not in config:
        raise KeyError(
            "Configuration is missing 'fleet' key. "
            "Please ensure your mission_config.yaml includes a fleet definition."
        )
    
    # Look up the drone's configuration
    fleet = config['fleet']
    if drone_id not in fleet:
        available_drones = ', '.join(fleet.keys())
        raise ValueError(
            f"Drone '{drone_id}' not found in fleet configuration. "
            f"Available drones: {available_drones}"
        )
    
    drone_config = fleet[drone_id]
    
    # Get the HAL type from the drone's configuration
    hal_type = drone_config.get('hal', 'simulated').lower()
    
    # Extract HAL-specific configuration
    # For some HALs, we may want to pass network config, camera URLs, etc.
    hal_config = {
        'drone_id': drone_id,
        'role': drone_config.get('role', 'unknown'),
        'plugins': drone_config.get('plugins', []),
    }
    
    # Add network configuration if available
    if 'network' in config:
        hal_config['network'] = config['network']
    
    # Add simulation configuration if available (for simulated HAL)
    if 'simulation' in config:
        hal_config['simulation'] = config['simulation']
    
    # Instantiate the appropriate HAL based on type
    if hal_type == 'simulated':
        # SimulatedController uses simulation config for initial position, speed, etc.
        sim_config = config.get('simulation', {})
        controller_config = {
            **hal_config,
            'flight_speed_ms': sim_config.get('flight_speed_ms', 10.0),
            'start_lat': sim_config.get('start_lat', 0.0),
            'start_lon': sim_config.get('start_lon', 0.0),
            'start_alt': sim_config.get('start_alt', 0.0),
        }
        return SimulatedController(controller_config)
    
    elif hal_type == 'ardupilot':
        # ArduPilotController has a different signature
        # It needs vehicle_state passed explicitly and expects connection_string in config
        vehicle_state = VehicleState()
        
        ardupilot_config = {
            **hal_config,
            'connection_string': config.get('network', {}).get('ardupilot_connection', 'udp:192.168.2.1:14550'),
            'camera_stream_url': config.get('network', {}).get('camera_stream_url', 'rtsp://192.168.2.1:554/stream'),
        }
        
        # Note: ArduPilotController has a non-standard signature
        # This is a known inconsistency in the codebase
        controller = ArduPilotController(vehicle_state, ardupilot_config)
        return controller
    
    elif hal_type == 'dji':
        dji_config = {
            **hal_config,
            'app_key': config.get('network', {}).get('dji_app_key', 'YOUR_DJI_APP_KEY'),
        }
        return DJIController(dji_config)
    
    elif hal_type == 'omniverse':
        omniverse_config = {
            **hal_config,
            'connection_string': config.get('network', {}).get('omniverse_connection', 'localhost:2318'),
        }
        return OmniverseController(omniverse_config)
    
    else:
        raise ValueError(
            f"Unknown HAL type: '{hal_type}'. "
            f"Supported types: simulated, ardupilot, dji, omniverse"
        )


# Export the base class and factory function
__all__ = [
    'BaseFlightController',
    'get_flight_controller'
]