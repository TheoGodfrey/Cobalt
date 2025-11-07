"""
Component 4: Behavior
Task execution logic.

This file defines the abstract BaseBehavior and concrete implementations
(e.g., SearchBehavior, DeliveryBehavior) as specified in the COBALT architecture.
Behaviors are instantiated by the MissionController and receive their
dependencies (plugins, HAL) via dependency injection.

REFACTOR: Added hardware_list dependency injection.
"""

import asyncio
import time # <-- NEW
from abc import ABC, abstractmethod
from typing import Protocol, Optional, Any, Dict, List # NEW: Added List

# --- Import correct data structures instead of placeholders ---
from .mission_state import MissionState, MissionStateEnum
from ..g1_mission_definition.phase import Task
from ..g3_capability_plugins.detectors.base import Detection
from ..g3_capability_plugins.strategies.base import Waypoint
from ..cross_cutting.communication import MqttClient # <-- NEW

# FIX for Bug #6: Import the factory functions from Bug #2
from ..g3_capability_plugins import get_detector, get_strategy, get_actuator

# --- Dependency Interface Definitions (using Protocols) ---
# ... (existing protocols Detector, Strategy, Actuator, HAL) ...
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
    
    # FIX: Add all methods from hal.py that are used
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
                 mqtt: MqttClient,    # <-- NEW
                 drone_id: str):      # <-- NEW
        """
        Initialize the behavior with its required dependencies.
        
        Args:
            mission_state: The shared MissionState object.
            hal: The Hardware Abstraction Layer.
            dependencies: A dictionary of required plugins.
            mqtt: The MqttClient for communication.
            drone_id: The drone's unique ID.
        """
        self.mission_state = mission_state
        self.hal = hal
        self.mqtt = mqtt      # <-- NEW
        self.drone_id = drone_id  # <-- NEW
        self._running = False
        self._task: Optional[asyncio.Task] = None
        
        # Assign injected dependencies
        self.detector: Optional[Detector] = dependencies.get("detector")
        self.strategy: Optional[Strategy] = dependencies.get("strategy")
        self.actuator: Optional[Actuator] = dependencies.get("actuator")

        # --- NEW: Failure tracking (Corrected) ---
        self.MAX_CONSECUTIVE_FAILURES = 3 # Max retries
        # Use a dictionary to track failures by type
        self._failure_counts: Dict[str, int] = {}
        # --- End of NEW ---

    async def start(self):
        """Starts the behavior's run loop as an async task."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self.run())
        # FIX: Don't await the task here, let it run in the background
        # await self._task 
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

    # --- NEW: Failure handling methods (Corrected) ---
    def _reset_failures(self, failure_type: str):
        """Resets the consecutive failure count for a specific type."""
        if self._failure_counts.get(failure_type, 0) > 0:
            print(f"[{self.__class__.__name__}] Action '{failure_type}' successful, "
                  f"resetting failure count.")
            self._failure_counts[failure_type] = 0

    def _increment_failure(self, failure_type: str = "unknown"):
        """Increments failure count for a type and triggers failure state if threshold is met."""
        # Increment the count for this specific failure type
        current_failures = self._failure_counts.get(failure_type, 0) + 1
        self._failure_counts[failure_type] = current_failures
        
        print(f"[{self.__class__.__name__}] WARNING: A {failure_type} failure occurred "
              f"({current_failures}/{self.MAX_CONSECUTIVE_FAILURES}).")
        
        if current_failures >= self.MAX_CONSECUTIVE_FAILURES:
            print(f"[{self.__class__.__name__}] CRITICAL: Failure threshold reached for '{failure_type}'. "
                  f"Transitioning to failure state.")
            
            # --- Transition to the correct failure state ---
            if failure_type == "navigation":
                self.mission_state.transition(MissionStateEnum.NAVIGATION_FAILURE)
            elif failure_type == "actuator":
                self.mission_state.transition(MissionStateEnum.ACTUATOR_FAILURE)
            else:
                # Fallback to a general abort if type is unknown
                print(f"[{self.__class__.__name__}] Unknown failure type '{failure_type}', ABORTING.")
                self.mission_state.transition(MissionStateEnum.ABORTED)
    # --- End of NEW ---

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
    
    --- REFACTORED to run sensing and flight loops concurrently. ---
    """

    async def _handle_detection(self, detection: Detection):
        """Helper to process and publish a detection."""
        print(f"[SearchBehavior] *** Target Found! *** at {detection.position}")
        
        # --- NEW: Publish detection to Hub ---
        try:
            # Create a simple track_id if one isn't provided
            track_id = detection.track_id or f"trk_{int(detection.timestamp)}"
            
            payload = {
                "drone_id": self.drone_id,
                "detection": {
                    "class_label": detection.class_label,
                    "confidence": detection.confidence,
                    "position": detection.position, # (pixel_x, pixel_y)
                    "timestamp": detection.timestamp,
                    "track_id": track_id
                }
            }
            
            if self.mqtt.is_connected():
                await self.mqtt.publish(
                    f"fleet/detections/{self.drone_id}", 
                    payload
                )
            print(f"[SearchBehavior] Published detection {track_id} to hub.")
        except Exception as e:
            print(f"[SearchBehavior] WARNING: Failed to publish detection: {e}")
        # --- End of NEW ---

    async def run(self):
        """
        Runs the flight and sensing loops concurrently.
        """
        print(f"[SearchBehavior] Running (Concurrent Mode)...")
        if not self.strategy or not self.detector:
            print("[SearchBehavior] Error: Missing Strategy or Detector.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        _target_found_event = asyncio.Event()

        async def _sensing_loop():
            """Continuously scans for targets."""
            print("[SearchBehavior] Sensing loop started...")
            try:
                while self._running and not _target_found_event.is_set():
                    # 1. Check for pause state
                    if self.mission_state.current == MissionStateEnum.PAUSED:
                        await asyncio.sleep(1)
                        continue
                    
                    # 2. Run detector
                    detection = await self.detector.detect()
                    
                    # 3. Handle detection
                    if detection and not _target_found_event.is_set():
                        await self._handle_detection(detection)
                        _target_found_event.set() # Signal flight loop to stop
                        break
                    
                    # 4. Yield control
                    await asyncio.sleep(0.05) # Fast scan rate
            except asyncio.CancelledError:
                print("[SearchBehavior] Sensing loop cancelled.")
            except Exception as e:
                print(f"[SearchBehavior] CRITICAL ERROR in sensing loop: {e}")
                self._increment_failure("detector_failure")
                _target_found_event.set() # Stop the behavior
            finally:
                print("[SearchBehavior] Sensing loop finished.")

        async def _flight_loop():
            """Continuously flies the search pattern."""
            print("[SearchBehavior] Flight loop started...")
            try:
                while self._running and not _target_found_event.is_set():
                    # 1. Check for pause state
                    if self.mission_state.current == MissionStateEnum.PAUSED:
                        await asyncio.sleep(1)
                        continue
                    
                    # 2. Get next waypoint
                    waypoint = await self.strategy.next_waypoint()
                    print(f"[SearchBehavior] Moving to waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})")

                    # 3. Start flying, but also watch for the target_found event
                    goto_task = asyncio.create_task(self.hal.goto(waypoint))
                    event_wait_task = asyncio.create_task(_target_found_event.wait())
                    
                    done, pending = await asyncio.wait(
                        {goto_task, event_wait_task}, 
                        return_when=asyncio.FIRST_COMPLETED
                    )

                    if event_wait_task in done:
                        # Target found by sensing loop *during* flight
                        print("[SearchBehavior] Target found during flight. Cancelling goto.")
                        goto_task.cancel()
                        break # Exit flight loop

                    # If here, goto_task finished. Get its result.
                    arrival_success = await goto_task
                    
                    if not arrival_success:
                        print(f"[SearchBehavior] Failed to reach waypoint.")
                        self._increment_failure("navigation")
                        continue # Try next waypoint
                    
                    self._reset_failures("navigation")
            except asyncio.CancelledError:
                print("[SearchBehavior] Flight loop cancelled.")
            except Exception as e:
                print(f"[SearchBehavior] CRITICAL ERROR in flight loop: {e}")
                self._increment_failure("navigation")
                _target_found_event.set() # Stop the behavior
            finally:
                print("[SearchBehavior] Flight loop finished.")

        # --- Main execution of the concurrent loops ---
        try:
            sensing_task = asyncio.create_task(_sensing_loop())
            flight_task = asyncio.create_task(_flight_loop())
            
            # Wait for either task to complete (which signals the other)
            done, pending = await asyncio.wait(
                {sensing_task, flight_task}, 
                return_when=asyncio.FIRST_COMPLETED
            )
            
            # Cancel the other pending task
            for task in pending:
                task.cancel()

            # Ensure both tasks are fully awaited to clear exceptions
            await asyncio.gather(sensing_task, flight_task, return_exceptions=True)
            
            # If the target was found, transition state (unless we failed)
            if _target_found_event.is_set():
                if self.mission_state.current not in (MissionStateEnum.ABORTED, MissionStateEnum.NAVIGATION_FAILURE):
                    self.mission_state.transition(MissionStateEnum.CONFIRMING)
            
            print("[SearchBehavior] Concurrent run finished.")

        except asyncio.CancelledError:
            print("[SearchBehavior] Canceled.")
        except Exception as e:
            print(f"[SearchBehavior] CRITICAL ERROR in run setup: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[SearchBehavior] Exiting run method.")

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

        try:
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
                    self._increment_failure("navigation") 
                    continue
                
                self._reset_failures("navigation") # <-- SPECIFY "navigation"

                # 4. At drop point, command the injected Actuator to execute
                print(f"[DeliveryBehavior] At drop point, executing payload release...")
                release_success = await self.actuator.execute()
                
                if release_success:
                    print(f"[DeliveryBehavior] *** Payload Released Successfully! ***")
                    self._reset_failures("actuator") # <-- SPECIFY "actuator"
                    # This trigger will be caught by the MissionController
                    self.mission_state.transition(MissionStateEnum.RETURNING)
                else:
                    print(f"[DeliveryBehavior] Payload release failed!")
                    # Implement retry logic or abort
                    self._increment_failure("actuator") 
                
                break # Delivery is usually a one-shot action
        except asyncio.CancelledError:
            print("[DeliveryBehavior] Canceled.")
        except Exception as e:
            print(f"[DeliveryBehavior] CRITICAL ERROR in run loop: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
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
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        try:
            home_waypoint = await self.strategy.next_waypoint()
            print(f"[RTHBehavior] Returning to Hub at: ({home_waypoint.x}, {home_waypoint.y}, {home_waypoint.z})")
            
            arrival_success = await self.hal.goto(home_waypoint) # <-- MODIFIED
            if not arrival_success:
                print("[RTHBehavior] Failed to reach home waypoint. Attempting failsafe land.")
                self._increment_failure("navigation") # Log the failure
                # Don't loop, just land
            else:
                self._reset_failures("navigation")
            
            print("[RTHBehavior] Arrived at Hub (or failed). Now landing...")
            
            # --- FIX: Actually call the land function ---
            await self.hal.land()
            # --- End of Fix ---
            
            print("[RTHBehavior] Landed.")
            self.mission_state.transition(MissionStateEnum.LANDING)
        except asyncio.CancelledError:
            print("[RTHBehavior] Canceled.")
        except Exception as e:
            print(f"[RTHBehavior] CRITICAL ERROR in run loop: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[RTHBehavior] Exiting run loop.")

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
    # NEW: Accept hardware_list in constructor
    def __init__(self, config: Dict[str, Any], hardware_list: List[str]):
        """
        Initializes the factory with configuration.
        
        Args:
            config: The complete mission configuration dictionary
            hardware_list: The list of available hardware on this drone
        """
        self.config = config
        self.hardware_list = hardware_list # NEW: Store hardware list
        print(f"[BehaviorFactory] Initialized with actual hardware: {self.hardware_list}")

    # NEW: Pass hardware_list to this method
    def _get_plugins(self, task: Task, hal: HAL, hardware_list: List[str]) -> Dict[str, Any]:
        """
        Instantiates the plugins required by a task using the factory functions.
        
        This is the FIX for Bug #6: Instead of just printing mock messages,
        this now actually creates plugin instances.
        
        Args:
            task: The Task definition from the mission JSON
            hal: The Hardware Abstraction Layer providing hardware interfaces
            hardware_list: The list of *actual* available hardware strings
            
        Returns:
            A dictionary of instantiated plugins: {"detector": ..., "strategy": ..., "actuator": ...}
        """
        dependencies = {}
        
        # --- Detector Loading ---
        if task.detector:
            try:
                print(f"[BehaviorFactory] Loading Detector: {task.detector}")
                camera = hal.get_camera(0)  # Use camera 0 by default
                
                # --- FIX: Get merged config from YAML and JSON ---
                detector_config = self._get_merged_plugin_config(
                    "detectors", task.detector, task.params
                )
                # --- End of FIX ---
                
                # NEW: Pass hardware_list to factory function
                detector = get_detector(
                    task.detector, 
                    camera, 
                    detector_config,
                    hardware_list
                )
                dependencies["detector"] = detector
                print(f"[BehaviorFactory] ✓ Detector '{task.detector}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown detector type '{task.detector}': {e}")
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load detector '{task.detector}': {e}")
                print(f"  (This may be a hardware validation failure from the plugin factory)")
        
        # --- Strategy Loading ---
        if task.strategy:
            try:
                print(f"[BehaviorFactory] Loading Strategy: {task.strategy}")
                vehicle_state = hal.vehicle_state
                
                # --- FIX: Get merged config from YAML and JSON ---
                strategy_config = self._get_merged_plugin_config(
                    "strategies", task.strategy, task.params
                )
                # --- End of FIX ---

                # NEW: Pass hardware_list to factory function
                strategy = get_strategy(
                    task.strategy, 
                    vehicle_state, 
                    strategy_config,
                    hardware_list
                )
                dependencies["strategy"] = strategy
                print(f"[BehaviorFactory] ✓ Strategy '{task.strategy}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown strategy type '{task.strategy}': {e}")
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load strategy '{task.strategy}': {e}")
                print(f"  (This may be a hardware validation failure from the plugin factory)")
        
        # --- Actuator Loading ---
        if task.actuator:
            try:
                print(f"[BehaviorFactory] Loading Actuator: {task.actuator}")
                actuator_hardware = hal.get_actuator_hardware(0)  # Use actuator 0 by default
                
                # --- FIX: Get merged config from YAML and JSON ---
                actuator_config = self._get_merged_plugin_config(
                    "actuators", task.actuator, task.params
                )
                # --- End of FIX ---
                
                # NEW: Pass hardware_list to factory function
                actuator = get_actuator(
                    task.actuator, 
                    actuator_hardware, 
                    actuator_config,
                    hardware_list
                )
                dependencies["actuator"] = actuator
                print(f"[BehaviorFactory] ✓ Actuator '{task.actuator}' loaded successfully")
                
            except ValueError as e:
                print(f"[BehaviorFactory] ✗ ERROR: Unknown actuator type '{task.actuator}': {e}")
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load actuator '{task.actuator}': {e}")
                print(f"  (This may be a hardware validation failure from the plugin factory)")
        
        return dependencies
    
    # --- FIX: Renamed and updated function to merge configs ---
    def _get_merged_plugin_config(self, 
                                 plugin_category: str, 
                                 plugin_name: str, 
                                 mission_params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Retrieves configuration for a plugin, merging system defaults
        with mission-specific parameters.

        Mission-specific parameters (from the .json) will
        overwrite system defaults (from the .yaml).
        
        Args:
            plugin_category: Category ("detectors", "strategies", "actuators")
            plugin_name: Name of the plugin (e.g., "goto_target")
            mission_params: The 'params' dictionary from the Task
            
        Returns:
            A final, merged configuration dictionary
        """
        
        # 1. Get default config from mission_config.yaml
        default_config = {}
        if plugin_category in self.config:
            if plugin_name in self.config[plugin_category]:
                default_config = self.config[plugin_category][plugin_name]
                # print(f"[BehaviorFactory] Loaded default config from '{plugin_category}.{plugin_name}'")

        # 2. Get mission-specific config from the .json 'params' block
        mission_specific_config = mission_params.get(plugin_name, {})
        if mission_specific_config:
             print(f"[BehaviorFactory] Found mission-specific config for '{plugin_name}'")

        # 3. Merge them (mission config overwrites defaults)
        final_config = {**default_config, **mission_specific_config}
        
        return final_config
    # --- End of FIX ---

    def create(self, 
                 task: Task, 
                 mission_state: MissionState, 
                 hal: HAL,
                 mqtt: MqttClient,    # <-- NEW
                 drone_id: str) -> BaseBehavior: # <-- NEW
        """
        Factory method to create a behavior instance.
        
        This is the main entry point for creating behaviors.
        It loads all required plugins and injects them into the behavior.
        
        Args:
            task: The Task definition from the mission JSON
            mission_state: The shared MissionState object
            hal: The Hardware Abstraction Layer
            mqtt: The MqttClient instance
            drone_id: The drone's unique ID
            
        Returns:
            An instance of a BaseBehavior subclass
            
        Raises:
            ValueError: If the task action is not recognized or plugins are missing
        """
        
        print(f"[BehaviorFactory] Creating behavior for action: {task.action}")
        
        # 1. Get the required plugins for this task
        # NEW: Pass the stored hardware_list
        dependencies = self._get_plugins(task, hal, self.hardware_list)
        
        # 2. Validate that required plugins were loaded
        self._validate_dependencies(task, dependencies)

        # 3. Instantiate the correct behavior based on the task action
        action = task.action.upper()
        
        if action == "EXECUTE_SEARCH":
            return SearchBehavior(mission_state, hal, dependencies, mqtt, drone_id) # <-- MODIFIED
            
        elif action == "EXECUTE_DELIVERY":
            return DeliveryBehavior(mission_state, hal, dependencies, mqtt, drone_id) # <-- MODIFIED
            
        elif action == "EXECUTE_RTH":
            return RTHBehavior(mission_state, hal, dependencies, mqtt, drone_id) # <-- MODIFIED
        
        # ... add other behaviors here ...
        
        else:
            # FIX: Add a default behavior for IGNORE
            if action == "IGNORE":
                # Return a dummy behavior that does nothing
                # We can re-use RTHBehavior with an empty dependency dict,
                # as it will just sit there (or use a dedicated NoOpBehavior)
                return RTHBehavior(mission_state, hal, {}, mqtt, drone_id) # <-- MODIFIED
            
            raise ValueError(f"Unknown behavior action: {task.action}")
    
    def _validate_dependencies(self, task: Task, dependencies: Dict[str, Any]):
        """
        Validates that all required plugins for a task were successfully loaded.
        
        Raises a ValueError if a required plugin is missing.
        
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
                    # --- MODIFIED: Raise error instead of printing ---
                    error_msg = (f"Missing required plugin: '{required}' is "
                                 f"needed for action '{action}' but was not loaded. "
                                 f"(Check hardware validation in main.py)")
                    print(f"[BehaviorFactory] ✗ ERROR: {error_msg}")
                    raise ValueError(error_msg)
                    # --- End of MODIFIED ---