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

# FIX for Bug #6: Import the factory functions from Bug #2
from ..g3_capability_plugins import get_detector, get_strategy, get_actuator

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


# ====================================================================================
# FIX for Bug #6: Complete BehaviorFactory Implementation
# ====================================================================================

class BehaviorFactory:
    """
    Instantiates and injects dependencies into the correct Behavior
    based on a task definition.
    
    This factory uses the plugin factory functions (Bug #2 fix) to
    actually load and configure plugins instead of just printing mock messages.
    """
    def __init__(self, config: Dict[str, Any]):
        """
        Initializes the factory with configuration.
        
        Args:
            config: The complete mission configuration dictionary
        """
        self.config = config
        print("[BehaviorFactory] Initialized with configuration.")

    def _get_plugins(self, task: Task, hal: HAL) -> Dict[str, Any]:
        """
        Instantiates the plugins required by a task using the factory functions.
        
        This is the FIX for Bug #6: Instead of just printing mock messages,
        this now actually creates plugin instances.
        
        Args:
            task: The Task definition from the mission JSON
            hal: The Hardware Abstraction Layer providing hardware interfaces
            
        Returns:
            A dictionary of instantiated plugins: {"detector": ..., "strategy": ..., "actuator": ...}
        """
        dependencies = {}
        
        # --- Detector Loading ---
        if task.detector:
            try:
                print(f"[BehaviorFactory] Loading Detector: {task.detector}")
                
                # Get camera hardware from HAL
                camera = hal.get_camera(0)  # Use camera 0 by default
                
                # Get detector-specific configuration
                detector_config = self._get_plugin_config("detectors", task.detector)
                
                # Use the factory function from Bug #2 fix
                detector = get_detector(task.detector, camera, detector_config)
                dependencies["detector"] = detector
                
                print(f"[BehaviorFactory] ✓ Detector '{task.detector}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown detector type '{task.detector}': {e}")
                # Don't add to dependencies if loading fails
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load detector '{task.detector}': {e}")
        
        # --- Strategy Loading ---
        if task.strategy:
            try:
                print(f"[BehaviorFactory] Loading Strategy: {task.strategy}")
                
                # Get vehicle state from HAL
                vehicle_state = hal.vehicle_state
                
                # Get strategy-specific configuration
                strategy_config = self._get_plugin_config("strategies", task.strategy)
                
                # Use the factory function from Bug #2 fix
                strategy = get_strategy(task.strategy, vehicle_state, strategy_config)
                dependencies["strategy"] = strategy
                
                print(f"[BehaviorFactory] ✓ Strategy '{task.strategy}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown strategy type '{task.strategy}': {e}")
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load strategy '{task.strategy}': {e}")
        
        # --- Actuator Loading ---
        if task.actuator:
            try:
                print(f"[BehaviorFactory] Loading Actuator: {task.actuator}")
                
                # Get actuator hardware from HAL
                actuator_hardware = hal.get_actuator_hardware(0)  # Use actuator 0 by default
                
                # Get actuator-specific configuration
                actuator_config = self._get_plugin_config("actuators", task.actuator)
                
                # Use the factory function from Bug #2 fix
                actuator = get_actuator(task.actuator, actuator_hardware, actuator_config)
                dependencies["actuator"] = actuator
                
                print(f"[BehaviorFactory] ✓ Actuator '{task.actuator}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown actuator type '{task.actuator}': {e}")
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load actuator '{task.actuator}': {e}")
        
        return dependencies
    
    def _get_plugin_config(self, plugin_category: str, plugin_name: str) -> Dict[str, Any]:
        """
        Retrieves configuration for a specific plugin from the main config.
        
        This method looks for plugin configuration in the following locations (in order):
        1. config[plugin_category][plugin_name]  (e.g., config["strategies"]["lawnmower"])
        2. config["plugins"][plugin_name]         (e.g., config["plugins"]["lawnmower"])
        3. Empty dict (default)
        
        Args:
            plugin_category: Category of plugin ("detectors", "strategies", "actuators")
            plugin_name: Name of the specific plugin (e.g., "thermal_detector")
            
        Returns:
            Configuration dictionary for the plugin
        """
        # Try category-specific config first
        if plugin_category in self.config:
            if plugin_name in self.config[plugin_category]:
                config = self.config[plugin_category][plugin_name]
                print(f"[BehaviorFactory] Using config from '{plugin_category}.{plugin_name}'")
                return config
        
        # Try generic plugins section
        if "plugins" in self.config:
            if plugin_name in self.config["plugins"]:
                config = self.config["plugins"][plugin_name]
                print(f"[BehaviorFactory] Using config from 'plugins.{plugin_name}'")
                return config
        
        # Default to empty config
        print(f"[BehaviorFactory] No config found for '{plugin_name}', using defaults")
        return {}

    def create(self, 
                 task: Task, 
                 mission_state: MissionState, 
                 hal: HAL) -> BaseBehavior:
        """
        Factory method to create a behavior instance.
        
        This is the main entry point for creating behaviors.
        It loads all required plugins and injects them into the behavior.
        
        Args:
            task: The Task definition from the mission JSON
            mission_state: The shared MissionState object
            hal: The Hardware Abstraction Layer
            
        Returns:
            An instance of a BaseBehavior subclass
            
        Raises:
            ValueError: If the task action is not recognized
        """
        
        print(f"[BehaviorFactory] Creating behavior for action: {task.action}")
        
        # 1. Get the required plugins for this task
        # This now actually loads plugins (Bug #6 fix)
        dependencies = self._get_plugins(task, hal)
        
        # 2. Validate that required plugins were loaded
        self._validate_dependencies(task, dependencies)

        # 3. Instantiate the correct behavior based on the task action
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
    
    def _validate_dependencies(self, task: Task, dependencies: Dict[str, Any]):
        """
        Validates that all required plugins for a task were successfully loaded.
        
        Logs warnings for missing plugins but doesn't raise errors, allowing
        behaviors to fail gracefully during execution.
        
        Args:
            task: The Task definition
            dependencies: The loaded dependencies
        """
        action = task.action.upper()
        
        # Define required dependencies for each action
        requirements = {
            "EXECUTE_SEARCH": ["detector", "strategy"],
            "EXECUTE_DELIVERY": ["strategy", "actuator"],
            "EXECUTE_RTH": ["strategy"],
        }
        
        if action in requirements:
            for required in requirements[action]:
                if required not in dependencies or dependencies[required] is None:
                    print(f"[BehaviorFactory] ⚠ WARNING: Required plugin '{required}' not loaded for action '{action}'")
                    print(f"[BehaviorFactory]   The behavior may fail during execution.")