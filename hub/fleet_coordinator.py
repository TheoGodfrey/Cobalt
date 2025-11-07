"""
Fleet Coordinator
This is the "brain" of the hub. It acts as the master
Mission Controller (Component 3) for the entire fleet.

It listens to all drones, manages fleet-wide state,
and issues commands based on the master mission plan.
"""

import asyncio
import os
from pathlib import Path
# FIX for Bug #10: Import asdict
from dataclasses import dataclass, field, asdict
import time 
from typing import Dict, Any, Optional, Set, List

# We need the communication layer
from drone.core.cross_cutting.communication import MqttClient

# We need the mission definition files (G1) to load
from drone.core.g1_mission_definition.loader import load_mission_file
# FIX for Bug #9: Function is parse_mission_flow, not parse_mission
from drone.core.g1_mission_definition.parser import parse_mission_flow, MissionParseError
from drone.core.g1_mission_definition.mission_flow import MissionFlow

# --- FIX 3.2: Added 'role' to DroneState ---
# Placeholder for a drone's state as tracked by the hub
@dataclass
class DroneState:
    drone_id: str
    role: str = "unknown" # Added role
    last_telemetry: Dict[str, Any] = field(default_factory=dict)
    mission_state: str = "IDLE"
    last_heartbeat: float = 0.0
# --- End of FIX 3.2 ---

class FleetCoordinator:
    """
    Manages the entire fleet of drones, assigns tasks,
    and acts as the central Mission Controller.
    """
    
    # --- FIX 3.2: Modified constructor to accept fleet config ---
    # This now matches the call from hub_main.py, fixing the TypeError
    def __init__(self, comms: MqttClient, fleet_config: Dict[str, Any]):
        self.comms = comms
        self.fleet_config = fleet_config # Store fleet config
        self.fleet_state: Dict[str, DroneState] = {}
        self.current_mission: Optional[MissionFlow] = None
        self.active_detections: Dict[str, Any] = {}
        self.confirmed_target: Optional[Dict[str, Any]] = None
        
        # --- NEW: Define a safe, absolute path for the missions directory ---
        # Assumes the hub is run from the project's root directory
        self.missions_dir = Path("missions").resolve()
        print(f"[FleetCoordinator] Initialized. Mission directory set to: {self.missions_dir}")
    # --- End of FIX 3.2 ---

    # --- FIX 3.2: Modified to populate role on first contact ---
    async def _handle_telemetry(self, topic: str, payload: Dict[str, Any]): # <-- MODIFIED
        """Callback for processing telemetry messages."""
        try:
            drone_id = payload['drone_id'] # <-- MODIFIED
            reported_state = payload['mission_state'] # <-- Get reported state
            
            is_new_drone = drone_id not in self.fleet_state
            
            if is_new_drone:
                # Get role from config, default to 'unknown'
                role = self.fleet_config.get(drone_id, {}).get('role', 'unknown')
                self.fleet_state[drone_id] = DroneState(drone_id=drone_id, role=role)
                print(f"[FleetCoordinator] New drone connected: {drone_id} (Role: {role})")

            # Update the drone's state in our fleet list
            state = self.fleet_state[drone_id]
            state.last_telemetry = payload['telemetry'] # <-- MODIFIED
            state.mission_state = reported_state
            state.last_heartbeat = time.monotonic()
            
            # --- NEW: Race condition fix ---
            # If a mission is loaded and this drone is IDLE,
            # assign it the starting phase.
            if self.current_mission and reported_state == "IDLE":
                print(f"[FleetCoordinator] Drone {drone_id} is IDLE. Assigning start phase: {self.current_mission.start_phase}")
                await self.assign_phase(self.current_mission.start_phase, drone_id)
            # --- End of NEW ---
            
            # print(f"[FleetCoordinator] Telemetry from {drone_id}: {state.mission_state}")
        except KeyError as e:
            print(f"[FleetCoordinator] Malformed telemetry message: {e}")
    # --- End of FIX 3.2 ---

    async def _handle_detections(self, topic: str, payload: Dict[str, Any]): # <-- MODIFIED
        """
        Handles incoming detections. Implements Hub-Mediated Consensus.
        (From Spec Appendix A.2)
        """
        try:
            # ... existing _handle_detections logic ...
            drone_id = payload['drone_id'] # <-- MODIFIED
            detection = payload['detection'] # <-- MODIFIED
            detection_id = detection['track_id']
            
            print(f"[FleetCoordinator] Received detection {detection_id} from {drone_id}")
            self.active_detections[detection_id] = detection

            # --- Hub-Mediated Consensus Logic (Simplified) ---
            # If we have 2+ detections of the same track, confirm it.
            # A real system would use spatial clustering.
            if len(self.active_detections) >= 2 and not self.confirmed_target:
                print(f"[FleetCoordinator] CONSENSUS: Target confirmed at {detection['position']}")
                self.confirmed_target = detection
                
                # Broadcast the confirmed target to all drones
                if self.comms.is_connected(): # <-- ADD THIS CHECK
                    await self.comms.publish("fleet/target_confirmed", {
                        "position": self.confirmed_target['position'],
                        "confidence": 1.0,
                        "supporting_drones": list(self.active_detections.keys())
                    })
                
                # Tell drones to switch to delivery phase (simplified)
                # --- NEW: Call assign_phase for *all* drones ---
                await self.assign_phase("delivery") # No drone_id means "all"

        except KeyError as e:
            print(f"[FleetCoordinator] Malformed detection message: {e}")

    async def _handle_drone_lwt(self, drone_id: str):
        """Handles a drone disconnecting (Last Will & Testament)."""
        if drone_id in self.fleet_state:
            print(f"[FleetCoordinator] Drone {drone_id} DISCONNECTED (LWT)")
            self.fleet_state[drone_id].mission_state = "OFFLINE"
            # TODO: Trigger re-allocation of tasks
            
    async def listen(self):
        """Main listening loop."""
        print("[FleetCoordinator] Starting listener tasks...")
        
        # Subscribe to all telemetry
        asyncio.create_task(self.comms.subscribe("fleet/telemetry/+", self._handle_telemetry))
        
        # Subscribe to all detections
        asyncio.create_task(self.comms.subscribe("fleet/detections/+", self._handle_detections))
        
        # Subscribe to drone disconnections (Bug #13)
        asyncio.create_task(self.comms.subscribe_lwt(self._handle_drone_lwt))

    async def load_and_start_mission(self, mission_filename: str):
        """Loads a mission JSON and commands drones to start."""
        print(f"[FleetCoordinator] Received request to load mission: {mission_filename}")
        try:
            # --- NEW: Path Traversal Security Fix ---
            if not mission_filename or ".." in mission_filename:
                raise ValueError("Invalid mission filename.")
                
            # Resolve the *absolute* path of the requested mission
            mission_path = (self.missions_dir / mission_filename).resolve()
            
            # Check that the resolved path is still *inside* the missions_dir
            # This prevents path traversal attacks like "../config/system_config.yaml"
            if self.missions_dir not in mission_path.parents:
                raise SecurityException(
                    f"Path Traversal Denied: {mission_filename} resolves outside missions directory."
                )
            if not mission_path.is_file():
                raise FileNotFoundError(f"Mission file not found at {mission_path}")
            # --- End of Security Fix ---
            
            # Get all unique, valid roles defined in the fleet config
            valid_roles = list(set(
                d.get('role') for d in self.fleet_config.values() if d.get('role')
            ))

            from drone.core.g1_mission_definition.loader import load_mission_file
            from drone.core.g1_mission_definition.parser import parse_mission_flow
            
            # --- MODIFIED: Load from the secured path ---
            mission_dict = load_mission_file(mission_path) 
            self.current_mission = parse_mission_flow(mission_dict, valid_roles)
            
        # --- THIS IS THE FIX ---
        # Exceptions are now re-raised so the GCS server can catch them
        except (ValueError, FileNotFoundError, SecurityException, MissionParseError) as e:
            print(f"[FleetCoordinator] FAILED to load mission: {e}")
            raise # Re-raise the exception
        except Exception as e:
            print(f"[FleetCoordinator] FAILED to load mission (unexpected error): {e}")
            raise # Re-raise the exception
        # --- END OF FIX ---

        print(f"[FleetCoordinator] Mission '{self.current_mission.mission_id}' loaded.")
        print(f"[FleetCoordinator] Hub is now WAITING for drones to connect and report IDLE.")
        
        # --- REMOVED: Do not assign phase here. Wait for drones. ---
        # await self.assign_phase(self.current_mission.start_phase)

    # --- FIX 3.2: Incorrect Role-Based Task Assignment ---
    # This function now assigns tasks based on the drone's role.
    async def assign_phase(self, phase_name: str, target_drone_id: Optional[str] = None):
        """
        Commands drones to execute a specific phase.
        If target_drone_id is specified, only commands that drone.
        Otherwise, commands all connected drones.
        """
        if not self.current_mission:
            print("[FleetCoordinator] No mission loaded, cannot assign phase.")
            return

        phase = self.current_mission.phases.get(phase_name)
        if not phase:
            print(f"[FleetCoordinator] Unknown phase '{phase_name}'")
            return
            
        # --- NEW: Determine which drones to command ---
        drones_to_command: List[DroneState] = []
        if target_drone_id:
            if target_drone_id in self.fleet_state:
                drones_to_command = [self.fleet_state[target_drone_id]]
            else:
                print(f"[FleetCoordinator] Cannot assign phase: Target drone {target_drone_id} not in fleet state.")
                return
        else:
            drones_to_command = list(self.fleet_state.values())
        # --- End of NEW ---
            
        print(f"[FleetCoordinator] Assigning phase '{phase_name}' to {len(drones_to_command)} drone(s).")
        
        # This is the "orchestration" step
        # It now assigns tasks based on role.
        for drone_state in drones_to_command: # <-- MODIFIED
            # 1. Get the drone's info
            drone_id = drone_state.drone_id
            drone_role = drone_state.role
            
            # 2. Look up the correct task for that role
            task = phase.tasks.get(drone_role)
            
            # 3. Send the specific task
            if task:
                print(f"  -> Sending task '{task.action}' to {drone_id} (Role: {drone_role})")
                command = {
                    "action": "EXECUTE_PHASE",
                    "phase": phase_name,
                    "task": asdict(task) # Send task config
                }
                if self.comms.is_connected(): # <-- ADD THIS CHECK
                    await self.comms.publish(f"fleet/commands/{drone_id}", command)
            else:
                # No task defined for this role in this phase (e.g., "payload" drone during "scout" phase)
                print(f"  -> No task for {drone_id} (Role: {drone_role}) in this phase. Sending IGNORE.")
                # We can send an explicit IGNORE or just send nothing.
                # Sending an explicit command is better for state management.
                command = {
                    "action": "EXECUTE_PHASE",
                    "phase": phase_name,
                    "task": {"action": "IGNORE"} 
                }
                if self.comms.is_connected(): # <-- ADD THIS CHECK
                    await self.comms.publish(f"fleet/commands/{drone_id}", command)
    # --- End of FIX 3.2 ---


    # --- FIX 1.4: GcsServer JSON Serialization TypeError ---
    # This method now uses asdict() to ensure the DroneState objects
    # are converted to JSON-serializable dictionaries before being sent.
    async def get_fleet_snapshot(self) -> Dict[str, Any]:
        """Gets a serializable snapshot of the fleet for the GCS."""
        return {
            "mission_id": self.current_mission.mission_id if self.current_mission else "NONE",
            "confirmed_target": self.confirmed_target,
            # FIX: Use asdict() to make states serializable
            "drones": [asdict(state) for state in self.fleet_state.values()]
        }
    # --- End of FIX 1.4 ---

# --- NEW: Custom Security Exception ---
class SecurityException(Exception):
    """Custom exception for security violations like path traversal."""
    pass