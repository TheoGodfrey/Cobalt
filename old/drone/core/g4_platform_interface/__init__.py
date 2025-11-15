"""
Platform Interface Package
This package isolates all hardware-specific code (G4).

It makes the various HAL implementations available
for easy import by the main application.
"""

# Expose the base class
from .hal import BaseFlightController

# Expose the concrete implementations
from .hal_simulated import SimulatedController
from .hal_ardupilot import ArduPilotController
from .hal_dji import DJIController
from .hal_omniverse import OmniverseController

# Expose Component 10
from .vehicle_state import VehicleState, Telemetry, VehicleModeEnum, GPSStatusEnum
