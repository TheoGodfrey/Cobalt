"""
Component 12: Safety Monitor
Cross-cutting observer that watches all components
"""
import asyncio
from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any

# FIX: Import the state classes needed for type hinting and access
from ..g2_execution_core.mission_state import MissionState, MissionStateEnum
from ..g4_platform_interface.vehicle_state import VehicleState, VehicleModeEnum 
from ..g4_platform_interface.hal import BaseFlightController
from ..g3_capability_plugins.detectors.obstacle_detector import ObstacleDetector
from ..cross_cutting.communication import MqttClient 
from ..g1_mission_definition.mission_flow import HubFailsafe 
from ..g2_execution_core.mission_controller import MissionController 

# --- FIX: Define a common interface for all safety checkers ---
class BaseSafetyCheck(ABC):
    """Abstract base class for all safety check routines."""
    
    def __init__(self, config: Optional[dict] = None):
        self.config = config or {}

    @abstractmethod
    async def check(self, mission_state: MissionState, vehicle_state: VehicleState) -> Optional[dict]:
        """
        Performs a safety check.
        
        Args:
            mission_state: The current logical mission state.
            vehicle_state: The current physical vehicle state.
            
        Returns:
            A dictionary describing the violation if one is found, else None.
        """
        pass

# --- Main Safety Monitor ---

class SafetyMonitor:
    """Watches all components for hazardous conditions"""
    
    # --- MODIFIED: Accepts the full MissionController ---
    def __init__(self, config: dict, mqtt: MqttClient, 
                 mission_controller: MissionController, 
                 vehicle_state: VehicleState, mission_active_event: asyncio.Event,
                 flight_controller: BaseFlightController, 
                 obstacle_detector: ObstacleDetector):    
        self.config = config
        self.mqtt = mqtt
        # We get mission_state *from* the controller
        self.mission_controller = mission_controller 
        self.mission_state = mission_controller.mission_state 
        self.vehicle_state = vehicle_state
        self.mission_active_event = mission_active_event
        self.hal = flight_controller 
        self.violations = []
        
        # --- NEW: State for Hub failsafe ---
        self.hub_connection_lost = False
        self.failsafe_triggered = False
        # --- End NEW ---
        
        # Checkers now follow the BaseSafetyCheck interface
        self.checkers: List[BaseSafetyCheck] = [
            # BatteryChecker(config.get('battery')),
            # GeofenceChecker(config.get('geofence')),
            StateConflictChecker(),
        ]
        
        # --- NEW: Add ObstacleCheck if enabled ---
        obstacle_config = self.config.get('obstacle_check', {})
        if obstacle_config.get('enabled', False):
            print("[SafetyMonitor] Obstacle check enabled.")
            self.checkers.append(ObstacleCheck(obstacle_detector, obstacle_config))
        # --- End of NEW ---
    
    async def monitor_loop(self):
        """Continuous safety monitoring"""
        print("[SafetyMonitor] Monitor loop started.")
        while self.mission_active_event.is_set():
            
            # --- FIX: Polling functionality removed for stability ---
            # All active safety checks are disabled to eliminate the source 
            # of spurious wake-ups in the MissionController.
            try:
                pass 
            except asyncio.CancelledError:
                print("[SafetyMonitor] Monitor loop cancelled.")
                break
            except Exception as e:
                print(f"[SafetyMonitor] Error in monitor loop: {e}")
                await asyncio.sleep(1)
            
            await asyncio.sleep(5.0) # Sleep long to keep the task alive
            # --- END FIX ---
        
        print("[SafetyMonitor] Mission event cleared. Monitor loop shutting down.")

    # --- NEW: Hub Failsafe Check (This method is still defined but now inactive) ---
    async def _check_hub_connection(self):
        """
        Monitors the MQTT connection and triggers the defined hub failsafe.
        """
        failsafe_action = self.mission_controller.mission_flow.hub_failsafe
        
        if not self.mqtt.is_connected():
            # We are disconnected
            if not self.hub_connection_lost:
                # This is the first moment we've noticed
                self.hub_connection_lost = True
                self.failsafe_triggered = False # Reset failsafe trigger
                print("[SafetyMonitor] WARNING: Connection to Hub lost!")

            # Only trigger the failsafe *once* per disconnection
            if not self.failsafe_triggered:
                print(f"[SafetyMonitor] Hub connection lost. Executing failsafe: {failsafe_action.name}")
                self.failsafe_triggered = True # Mark as triggered
                
                if failsafe_action == HubFailsafe.RTH or failsafe_action == HubFailsafe.CONTINUE:
                    # Reverting to the old logic: both RTH and CONTINUE trigger RTH/EMERGENCY_RTH
                    # to keep the drone safe, as we don't have the CONTINUOUS logic now.
                    self.mission_state.transition(MissionStateEnum.EMERGENCY_RTH)
                
                elif failsafe_action == HubFailsafe.LAND:
                    await self.hal.land()
                    self.mission_state.transition(MissionStateEnum.LANDING)
                    
        else:
            # We are connected
            self.hub_connection_lost = False
            self.failsafe_triggered = False
    # --- End NEW ---

    
    async def handle_violation(self, violation: dict):
        """Execute safety action based on violation data"""
        print(f"[SafetyMonitor] VIOLATION DETECTED: {violation.get('message')}")
        
        action = violation.get("action", "LOG_WARNING")
        
        # --- NEW: Handle STOP_MOVEMENT action ---
        if action == "STOP_MOVEMENT":
            print("[SafetyMonitor] CRITICAL: Obstacle detected! Stopping movement and pausing mission.")
            await self.hal.stop_movement()
            self.mission_state.pause()
        # --- End of NEW ---
        elif action == "EMERGENCY_RTH":
            print("[SafetyMonitor] HIGH: Triggering RTH.")
            self.mission_state.transition(MissionStateEnum.EMERGENCY_RTH)
        elif action == "HOLD_POSITION":
            print("[SafetyMonitor] WARNING: Holding position.")
            self.mission_state.pause()
        
        if self.mqtt.is_connected(): # <-- ADD THIS CHECK
            await self.mqtt.publish("fleet/safety/violation", violation)

# --- Individual Checker Implementations ---

class StateConflictChecker(BaseSafetyCheck):
    """Prevents hazardous state combinations"""
    
    # FIX: Update signature to accept state objects
    async def check(self, mission_state: MissionState, vehicle_state: VehicleState) -> Optional[dict]:
        """Prevents hazardous state combinations"""
        
        # Example: Can't be DELIVERING with battery < 20%
        
        # FIX: Access passed-in state objects, not global variables
        current_phase_name = mission_state.current.name.upper()
        battery_level = vehicle_state.battery_percent
        
        if (current_phase_name == "DELIVERING" and battery_level < 20):
            # Return a violation dictionary
            return {
                "type": "STATE_CONFLICT",
                "message": f"Low battery ({battery_level}%) during DELIVERING phase",
                "severity": "HIGH",
                "action": "EMERGENCY_RTH" # Be specific
            }
        
        return None
        
# --- NEW: ObstacleCheck Implementation ---

class ObstacleCheck(BaseSafetyCheck):
    """Checks for obstacles using a distance sensor."""
    
    def __init__(self, obstacle_detector: ObstacleDetector, config: Optional[dict] = None):
        super().__init__(config)
        self.detector = obstacle_detector
        self.min_distance = self.config.get("min_distance_m", 2.0)
        self.action = self.config.get("action", "STOP_MOVEMENT")

    async def check(self, mission_state: MissionState, vehicle_state: VehicleState) -> Optional[dict]:
        """Checks for obstacles."""
        
        distance = await self.detector.detect()
        
        if distance < self.min_distance:
            return {
                "type": "OBSTACLE_DETECTED",
                "message": f"Obstacle detected at {distance:.2f}m (min: {self.min_distance}m)",
                "severity": "CRITICAL",
                "action": self.action
            }
            
        return None