"""
Component 4: Behavior
Task execution logic.

This file defines the abstract BaseBehavior and concrete implementations
(e.g., SearchBehavior, DeliveryBehavior) as specified in the COBALT architecture.
Behaviors are instantiated by the MissionController and receive their
dependencies (plugins, HAL) via dependency injection.
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Protocol, Optional, Any, Dict

# --- Import correct data structures instead of placeholders ---
from .mission_state import MissionState, MissionStateEnum
from ..g1_mission_definition.phase import Task
from ..g3_capability_plugins.detectors.base import Detection
from ..g3_capability_plugins.strategies.base import Waypoint

# --- Dependency Interface Definitions (using Protocols) ---
# These define the interfaces that Behaviors expect from other components
# (Detectors, Strategies, Actuators, HAL) without creating circular imports.

class Detector(Protocol):
    """Expected interface for a Detector plugin."""
    async def detect(self) -> Optional[Detection]:
        ...

class Strategy(Protocol):
    """Expected interface for a Strategy plugin."""
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
    
    async def get_position(self) -> Waypoint:
        ...

# --- Base Behavior ---

class BaseBehavior(ABC):
    """
    Abstract base class for all mission behaviors.
    """
    def __init__(self,
                 mission_state: MissionState,
                 hal: HAL,
                 dependencies: Dict[str, Any]):
        """
        Initialize the behavior with its required dependencies.
        
        Args:
            mission_state: The shared MissionState object.
            hal: The Hardware Abstraction Layer.
            dependencies: A dictionary of required plugins, e.g.,
                          {"detector": ThermalDetector(), "strategy": ProbSearch()}
        """
        self.mission_state = mission_state
        self.hal = hal
        self._running = False
        self._task: Optional[asyncio.Task] = None
        
        # Assign injected dependencies
        self.detector: Optional[Detector] = dependencies.get("detector")
        self.strategy: Optional[Strategy] = dependencies.get("strategy")
        self.actuator: Optional[Actuator] = dependencies.get("actuator")

    async def start(self):
        """Starts the behavior's run loop as an async task."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self.run())
        await self._task

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

    @abstractmethod
    async def run(self):
        """
        The main logic loop for the behavior.
        This method must be implemented by all concrete behaviors.
        It should periodically check `self._running`.
        """
        pass

# --- Concrete Behaviors ---

class SearchBehavior(BaseBehavior):
    """
    Executes a search task.
    Coordinates between a Strategy (to get waypoints) and a
    Detector (to find the target).
    """
    async def run(self):
        print(f"[SearchBehavior] Running...")
        if not self.strategy or not self.detector:
            print("[SearchBehavior] Error: Missing Strategy or Detector.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        while self._running:
            # 1. Check for pause state
            if self.mission_state.current == MissionStateEnum.PAUSED:
                await asyncio.sleep(1)
                continue
            
            # 2. Get next waypoint from the injected Strategy
            waypoint = await self.strategy.next_waypoint()
            print(f"[SearchBehavior] Moving to waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})")
            
            # 3. Command the HAL to move to the waypoint
            arrival_success = await self.hal.goto(waypoint)
            if not arrival_success:
                print(f"[SearchBehavior] Failed to reach waypoint.")
                # Implement retry logic or abort
                continue

            # 4. At waypoint, use the injected Detector to look for target
            print(f"[SearchBehavior] At waypoint, scanning for target...")
            detection = await self.detector.detect()
            
            if detection and detection.confidence > 0.9:
                print(f"[SearchBehavior] *** Target Found! *** at {detection.position}")
                # Transition mission state to CONFIRMING or DELIVERING
                # This trigger will be caught by the MissionController
                self.mission_state.transition(MissionStateEnum.CONFIRMING)
                break # Exit the search loop
            
            # 5. Check for stop signal
            if not self._running:
                break
            
            await asyncio.sleep(0.1) # Yield control

        print("[SearchBehavior] Exiting run loop.")

class DeliveryBehavior(BaseBehavior):
    """
    Executes a payload delivery task.
    Coordinates between a Strategy (to get drop point) and an
    Actuator (to release the payload).
    """
    async def run(self):
        print(f"[DeliveryBehavior] Running...")
        if not self.strategy or not self.actuator:
            print("[DeliveryBehavior] Error: Missing Strategy or Actuator.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        while self._running:
            # 1. Check for pause state
            if self.mission_state.current == MissionStateEnum.PAUSED:
                await asyncio.sleep(1)
                continue
                
            # 2. Get the delivery/drop-off waypoint from the Strategy
            drop_waypoint = await self.strategy.next_waypoint()
            print(f"[DeliveryBehavior] Moving to drop point: ({drop_waypoint.x}, {drop_waypoint.y}, {drop_waypoint.z})")

            # 3. Command the HAL to move to the drop point
            arrival_success = await self.hal.goto(drop_waypoint)
            if not arrival_success:
                print(f"[DeliveryBehavior] Failed to reach drop point.")
                continue

            # 4. At drop point, command the injected Actuator to execute
            print(f"[DeliveryBehavior] At drop point, executing payload release...")
            release_success = await self.actuator.execute()
            
            if release_success:
                print(f"[DeliveryBehavior] *** Payload Released Successfully! ***")
                # This trigger will be caught by the MissionController
                self.mission_state.transition(MissionStateEnum.RETURNING)
            else:
                print(f"[DeliveryBehavior] Payload release failed!")
                # Implement retry logic or abort
                self.mission_state.transition(MissionStateEnum.ABORTED)
            
            break # Delivery is usually a one-shot action

        print("[DeliveryBehavior] Exiting run loop.")

class RTHBehavior(BaseBehavior):
    """
    Executes a simple Return-to-Hub task.
    Uses strategy for a home waypoint and HAL to fly there.
    """
    async def run(self):
        print(f"[RTHBehavior] Running...")
        if not self.strategy:
            print("[RTHBehavior] Error: Missing Strategy for home position.")
            # Failsafe: might need a hardcoded RTH in HAL
            return

        home_waypoint = await self.strategy.next_waypoint()
        print(f"[RTHBehavior] Returning to Hub at: ({home_waypoint.x}, {home_waypoint.y}, {home_waypoint.z})")
        
        await self.hal.goto(home_waypoint)
        
        print("[RTHBehavior] Arrived at Hub.")
        self.mission_state.transition(MissionStateEnum.LANDING)

# Add other behaviors as needed (Patrol, Standby, etc.)


# --- Behavior Factory ---
# This class is the fix for Bug #1
# It is responsible for creating the correct behavior based on the mission task

class BehaviorFactory:
    """
    Instantiates and injects dependencies into the correct Behavior
    based on a task definition.
    """
    def __init__(self, config: Dict[str, Any]):
        """
        Initializes the factory.
        In a real system, this is where you would load all available
        plugins (detectors, strategies, actuators).
        """
        # For now, we'll create them on the fly.
        # A real system would use a plugin registry.
        self.config = config
        print("[BehaviorFactory] Initialized.")

    def _get_plugins(self, task: Task, hal: HAL) -> Dict[str, Any]:
        """
        A helper to instantiate the plugins required by a task.
        This is a simple implementation; a real one would be
        more robust, loading plugins by name from a registry.
        """
        dependencies = {}
        
        # --- This part is still simulated ---
        # A real implementation would look up these names in a
        # plugin registry and instantiate the correct class.
        
        if task.detector:
            # e.g., if task.detector == "thermal_detector":
            # from ..g3_capability_plugins.detectors.thermal_detector import ThermalDetector
            # dependencies["detector"] = ThermalDetector(hal.get_camera(0))
            print(f"[BehaviorFactory] Mock loading Detector: {task.detector}")
            
        if task.strategy:
            # e.g., if task.strategy == "lawnmower":
            # from ..g3_capability_plugins.strategies.lawnmower import LawnmowerStrategy
            # dependencies["strategy"] = LawnmowerStrategy(hal.vehicle_state, self.config.get('lawnmower_config'))
            print(f"[BehaviorFactory] Mock loading Strategy: {task.strategy}")

        if task.actuator:
            # e.g., if task.actuator == "payload_dropper":
            # from ..g3_capability_plugins.actuators.payload_dropper import PayloadDropper
            # dependencies["actuator"] = PayloadDropper(hal.get_actuator_hardware(0))
            print(f"[BehaviorFactory] Mock loading Actuator: {task.actuator}")
            
        # This is highly simplified. A real DI system is needed.
        return dependencies

    def create(self, 
                 task: Task, 
                 mission_state: MissionState, 
                 hal: HAL) -> BaseBehavior:
        """
        Factory method to create a behavior instance.
        """
        
        # 1. Get the required plugins for this task
        # This is a placeholder for a real dependency injection system
        dependencies = self._get_plugins(task, hal)

        # 2. Instantiate the correct behavior based on the task action
        action = task.action.upper()
        
        if action == "EXECUTE_SEARCH":
            return SearchBehavior(mission_state, hal, dependencies)
            
        elif action == "EXECUTE_DELIVERY":
            return DeliveryBehavior(mission_state, hal, dependencies)
            
        elif action == "EXECUTE_RTH":
            return RTHBehavior(mission_state, hal, dependencies)
        
        # ... add other behaviors here ...
        
        else:
            raise ValueError(f"Unknown behavior action: {task.action}")