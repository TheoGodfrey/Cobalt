"""
Component 4: Behavior (Base Interface)
Defines the abstract base class for all mission behaviors and their
dependent protocols.
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Protocol, Optional, Any, Dict

# --- Import correct data structures instead of placeholders ---
from ..mission_state import MissionState, MissionStateEnum
from ...g3_capability_plugins.detectors.base import Detection
from ...g3_capability_plugins.strategies.base import Waypoint
from ...cross_cutting.communication import MqttClient

# --- Dependency Interface Definitions (using Protocols) ---

class Detector(Protocol):
    """Expected interface for a Detector plugin."""
    async def detect(self) -> Optional[Detection]:
        ...

class Strategy(Protocol):
    """Expected interface for a Strategy plugin."""
    config: Dict[str, Any] # Add config to protocol
    async def next_waypoint(self) -> Waypoint:
        ...

class Actuator(Protocol):
    """Expected interface for an Actuator plugin."""
    async def execute(self) -> bool: # Returns True on success
        ...

class HAL(Protocol):
    """Expected interface for the Hardware Abstraction Layer."""
    async def goto(self, waypoint: Waypoint) -> bool: # Returns True on arrival
        ...
    
    async def land(self) -> bool:
        ...
    
    def get_camera(self, camera_id: int) -> Any: # Returns a Camera protocol object
        ...
        
    def get_actuator_hardware(self, actuator_id: int) -> Any: # Returns ActuatorHardware protocol
        ...
        
    @property
    def vehicle_state(self) -> Any: # Returns a VehicleState protocol object
        ...

# --- Base Behavior ---

class BaseBehavior(ABC):
    """
    Abstract base class for all mission behaviors.
    """
    def __init__(self,
                 mission_state: MissionState,
                 hal: HAL,
                 dependencies: Dict[str, Any],
                 mqtt: MqttClient,
                 drone_id: str,
                 config: Dict[str, Any],
                 drone_role: str): # <--- FIX: ADDED drone_role (7th explicit arg)
        """
        Initialize the behavior with its required dependencies.
        
        Args:
            mission_state: The shared MissionState object.
            hal: The Hardware Abstraction Layer.
            dependencies: A dictionary of required plugins.
            mqtt: The MqttClient for communication.
            drone_id: The drone's unique ID.
            config: The main system config dictionary.
            drone_role: The role (e.g., 'scout', 'payload') of this drone.
        """
        self.mission_state = mission_state
        self.hal = hal
        self.mqtt = mqtt
        self.drone_id = drone_id
        self.config = config
        self.drone_role = drone_role # <--- FIX: Store the role
        self._running = False
        self._task: Optional[asyncio.Task] = None
        
        # Assign injected dependencies
        self.detector: Optional[Detector] = dependencies.get("detector")
        self.strategy: Optional[Strategy] = dependencies.get("strategy")
        self.actuator: Optional[Actuator] = dependencies.get("actuator")

        # --- Failure tracking ---
        self.MAX_CONSECUTIVE_FAILURES = 3 # Max retries
        self._failure_counts: Dict[str, int] = {}

    async def start(self):
        """Starts the behavior's run loop as an async task."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self.run())
        print(f"[{self.__class__.__name__}] Started.")


    async def stop(self):
        """Stops the behavior's run loop."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._task = None
        print(f"[{self.__class__.__name__}] Stopped.")

    # --- Failure handling methods ---
    def _reset_failures(self, failure_type: str):
        """Resets the consecutive failure count for a specific type."""
        if self._failure_counts.get(failure_type, 0) > 0:
            print(f"[{self.__class__.__name__}] Action '{failure_type}' successful, "
                  f"resetting failure count.")
            self._failure_counts[failure_type] = 0

    def _increment_failure(self, failure_type: str = "unknown"):
        """Increments failure count for a type and triggers failure state if threshold is met."""
        current_failures = self._failure_counts.get(failure_type, 0) + 1
        self._failure_counts[failure_type] = current_failures
        
        print(f"[{self.__class__.__name__}] WARNING: A {failure_type} failure occurred "
              f"({current_failures}/{self.MAX_CONSECUTIVE_FAILURES}).")
        
        if current_failures >= self.MAX_CONSECUTIVE_FAILURES:
            print(f"[{self.__class__.__name__}] CRITICAL: Failure threshold reached for '{failure_type}'. "
                  f"Transitioning to failure state.")
            
            if failure_type == "navigation":
                self.mission_state.transition(MissionStateEnum.NAVIGATION_FAILURE)
            elif failure_type == "actuator":
                self.mission_state.transition(MissionStateEnum.ACTUATOR_FAILURE)
            else:
                print(f"[{self.__class__.__name__}] Unknown failure type '{failure_type}', ABORTING.")
                self.mission_state.transition(MissionStateEnum.ABORTED)

    @abstractmethod
    async def run(self):
        """
        The main logic loop for the behavior.
        This method must be implemented by all concrete behaviors.
        It should periodically check `self._running`.
        """
        pass