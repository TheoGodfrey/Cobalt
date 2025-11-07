"""
Component 12: Safety Monitor
Cross-cutting observer that watches all components
"""
import asyncio
from abc import ABC, abstractmethod
from typing import Optional

# FIX: Import the state classes needed for type hinting and access
from ..g2_execution_core.mission_state import MissionState
from ..g4_platform_interface.vehicle_state import VehicleState

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
    
    # FIX: Update __init__ to accept state objects
    def __init__(self, config: dict, mqtt, mission_state: MissionState, vehicle_state: VehicleState):
        self.config = config
        self.mqtt = mqtt
        self.mission_state = mission_state
        self.vehicle_state = vehicle_state
        self.violations = []
        
        # Checkers now follow the BaseSafetyCheck interface
        self.checkers = [
            # BatteryChecker(config.get('battery')),
            # GeofenceChecker(config.get('geofence')),
            StateConflictChecker(),
            # CollisionChecker()
        ]
    
    async def monitor_loop(self):
        """Continuous safety monitoring"""
        while True:
            for checker in self.checkers:
                # FIX: Pass the state objects to the checker
                violation = await checker.check(self.mission_state, self.vehicle_state)
                if violation:
                    await self.handle_violation(violation)
            
            # Check vehicle state for manual override
            if self.vehicle_state.mode == "MANUAL":
                # This is an example of a direct check
                if self.mission_state.current != "PAUSED":
                    print("[SafetyMonitor] Manual override detected! Pausing mission.")
                    self.mission_state.pause()

            await asyncio.sleep(0.1)
    
    async def handle_violation(self, violation: dict):
        """Execute safety action based on violation data"""
        print(f"[SafetyMonitor] VIOLATION DETECTED: {violation.get('message')}")
        
        severity = violation.get("severity", "HIGH")
        
        if severity == "CRITICAL":
            # await self.trigger_emergency_land()
            print("[SafetyMonitor] CRITICAL: Triggering Emergency Land (Simulated)")
        elif severity == "HIGH":
            # await self.trigger_rth()
            print("[SafetyMonitor] HIGH: Triggering RTH (Simulated)")
        
        if self.mqtt.is_connected():
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
                "severity": "HIGH"
            }
        
        return None

# Placeholder for other checkers (to show the pattern)
# class BatteryChecker(BaseSafetyCheck):
#     async def check(self, mission_state: MissionState, vehicle_state: VehicleState) -> Optional[dict]:
#         if vehicle_state.battery_percent < self.config.get("critical_threshold", 15):
#             return {
#                 "type": "LOW_BATTERY_CRITICAL",
#                 "message": f"Battery critically low: {vehicle_state.battery_percent}%",
#                 "severity": "CRITICAL"
#             }
#         return None