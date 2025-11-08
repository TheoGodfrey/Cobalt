"""
Component 4: Behavior (Factory)
Instantiates and injects dependencies into the correct Behavior
based on a task definition.
"""

from typing import Any, Dict, List

from ..mission_state import MissionState
from ...g1_mission_definition.phase import Task
from ...cross_cutting.communication import MqttClient

# Import the base class and protocols
from .base import BaseBehavior, HAL

# Import the factory functions for plugins
from ...g3_capability_plugins import get_detector, get_strategy, get_actuator

# Import all concrete behavior implementations
from .search_behavior import SearchBehavior
from .delivery_behavior import DeliveryBehavior
from .rth_behavior import RTHBehavior
from .sacrificial_delivery_behavior import SacrificialDeliveryBehavior
# from .investigate_behavior import InvestigateBehavior # (Example)


class BehaviorFactory:
    """
    Instantiates and injects dependencies into the correct Behavior
    based on a task definition.
    """
    def __init__(self, config: Dict[str, Any], hardware_list: List[str]):
        """
        Initializes the factory with configuration.
        
        Args:
            config: The complete mission configuration dictionary
            hardware_list: The list of available hardware on this drone
        """
        self.config = config
        self.hardware_list = hardware_list
        print(f"[BehaviorFactory] Initialized with actual hardware: {self.hardware_list}")

    def _get_plugins(self, task: Task, hal: HAL, hardware_list: List[str]) -> Dict[str, Any]:
        """
        Instantiates the plugins required by a task using the factory functions.
        """
        dependencies = {}
        
        # --- Detector Loading ---
        if task.detector:
            try:
                print(f"[BehaviorFactory] Loading Detector: {task.detector}")
                camera = hal.get_camera(0)
                detector_config = self._get_merged_plugin_config(
                    "detectors", task.detector, task.params
                )
                detector = get_detector(
                    task.detector, 
                    camera, 
                    detector_config,
                    hardware_list
                )
                dependencies["detector"] = detector
                print(f"[BehaviorFactory] ✓ Detector '{task.detector}' loaded successfully")
                
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load detector '{task.detector}': {e}")
        
        # --- Strategy Loading ---
        if task.strategy:
            try:
                print(f"[BehaviorFactory] Loading Strategy: {task.strategy}")
                vehicle_state = hal.vehicle_state
                strategy_config = self._get_merged_plugin_config(
                    "strategies", task.strategy, task.params
                )
                strategy = get_strategy(
                    task.strategy, 
                    vehicle_state, 
                    strategy_config,
                    hardware_list
                )
                dependencies["strategy"] = strategy
                print(f"[BehaviorFactory] ✓ Strategy '{task.strategy}' loaded successfully")
                
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load strategy '{task.strategy}': {e}")
        
        # --- Actuator Loading ---
        if task.actuator:
            try:
                print(f"[BehaviorFactory] Loading Actuator: {task.actuator}")
                actuator_hardware = hal.get_actuator_hardware(0)
                actuator_config = self._get_merged_plugin_config(
                    "actuators", task.actuator, task.params
                )
                actuator = get_actuator(
                    task.actuator, 
                    actuator_hardware, 
                    actuator_config,
                    hardware_list
                )
                dependencies["actuator"] = actuator
                print(f"[BehaviorFactory] ✓ Actuator '{task.actuator}' loaded successfully")
                
            except Exception as e:
                print(f"[BehaviorFactory] ✗ ERROR: Failed to load actuator '{task.actuator}': {e}")
        
        return dependencies
    
    def _get_merged_plugin_config(self, 
                                 plugin_category: str, 
                                 plugin_name: str, 
                                 mission_params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Retrieves configuration for a plugin, merging system defaults
        with mission-specific parameters.
        """
        default_config = {}
        if plugin_category in self.config:
            if plugin_name in self.config[plugin_category]:
                default_config = self.config[plugin_category][plugin_name]

        mission_specific_config = mission_params.get(plugin_name, {})
        if mission_specific_config:
             print(f"[BehaviorFactory] Found mission-specific config for '{plugin_name}'")

        final_config = {**default_config, **mission_specific_config}
        
        return final_config

    def create(self, 
                 task: Task, 
                 mission_state: MissionState, 
                 hal: HAL,
                 mqtt: MqttClient,
                 drone_id: str,
                 config: Dict[str, Any],
                 drone_role: str) -> BaseBehavior: # <--- FIX: ADDED drone_role (7th explicit arg)
        """
        Factory method to create a behavior instance.
        """
        
        print(f"[BehaviorFactory] Creating behavior for action: {task.action}")
        
        dependencies = self._get_plugins(task, hal, self.hardware_list)
        self._validate_dependencies(task, dependencies)

        action = task.action.upper()
        
        # NOTE: All behavior constructors now take the 7 arguments defined above.
        
        if action == "EXECUTE_SEARCH":
            return SearchBehavior(mission_state, hal, dependencies, mqtt, drone_id, config, drone_role)
            
        elif action == "EXECUTE_DELIVERY":
            return DeliveryBehavior(mission_state, hal, dependencies, mqtt, drone_id, config, drone_role)
            
        elif action == "EXECUTE_RTH":
            return RTHBehavior(mission_state, hal, dependencies, mqtt, drone_id, config, drone_role)
        
        elif action == "EXECUTE_SACRIFICIAL_DELIVERY":
            return SacrificialDeliveryBehavior(mission_state, hal, dependencies, mqtt, drone_id, config, drone_role)
        
        elif action == "IGNORE":
            # Return a behavior that does nothing but transitions to LANDING
            return RTHBehavior(mission_state, hal, {}, mqtt, drone_id, config, drone_role)
            
        else:
            raise ValueError(f"Unknown behavior action: {task.action}")
    
    def _validate_dependencies(self, task: Task, dependencies: Dict[str, Any]):
        """
        Validates that all required plugins for a task were successfully loaded.
        """
        action = task.action.upper()
        
        requirements = {
            "EXECUTE_SEARCH": ["detector", "strategy"],
            "EXECUTE_DELIVERY": ["strategy", "actuator"],
            "EXECUTE_RTH": ["strategy"],
            "EXECUTE_SACRIFICIAL_DELIVERY": ["strategy"],
        }
        
        if action in requirements:
            for required in requirements[action]:
                if required not in dependencies or dependencies[required] is None:
                    error_msg = (f"Missing required plugin: '{required}' is "
                                 f"needed for action '{action}' but was not loaded. "
                                 f"(Check hardware validation in main.py)")
                    print(f"[BehaviorFactory] ✗ ERROR: {error_msg}")
                    raise ValueError(error_msg)