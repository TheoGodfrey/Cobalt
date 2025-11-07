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

# --- THIS IS THE FIX ---
from drone.core.utils.position import LocalPosition
from .tracked_target import TrackedTarget, TargetStatus
# --- END OF FIX ---


@dataclass
class DroneState:
    drone_id: str
    role: str = "unknown" # Added role
    last_telemetry: Dict[str, Any] = field(default_factory=dict)
    mission_state: str = "IDLE"
    last_heartbeat: float = 0.0

class FleetCoordinator:
    """
    Manages the entire fleet of drones, assigns tasks,
    and acts as the central Mission Controller.
    """
    
    def __init__(self, comms: MqttClient, fleet_config: Dict[str, Any]):
        self.comms = comms
        self.fleet_config = fleet_config # Store fleet config
        self.fleet_state: Dict[str, DroneState] = {}
        self.current_mission: Optional[MissionFlow] = None
        
        # --- THIS IS THE FIX ---
        # Replace simple dicts with a new target tracking system
        self.tracked_targets: Dict[str, TrackedTarget] = {}
        self.confirmed_target_id: Optional[str] = None
        # --- END OF FIX ---
        
        self.missions_dir = Path("missions").resolve()
        print(f"[FleetCoordinator] Initialized. Mission directory set to: {self.missions_dir}")

    async def _handle_telemetry(self, topic: str, payload: Dict[str, Any]):
        """Callback for processing telemetry messages."""
        try:
            drone_id = payload['drone_id'] 
            reported_state = payload['mission_state']
            
            is_new_drone = drone_id not in self.fleet_state
            
            if is_new_drone:
                role = self.fleet_config.get(drone_id, {}).get('role', 'unknown')
                self.fleet_state[drone_id] = DroneState(drone_id=drone_id, role=role)
                print(f"[FleetCoordinator] New drone connected: {drone_id} (Role: {role})")

            state = self.fleet_state[drone_id]
            state.last_telemetry = payload['telemetry']
            state.mission_state = reported_state
            state.last_heartbeat = time.monotonic()
            
            if self.current_mission and reported_state == "IDLE":
                print(f"[FleetCoordinator] Drone {drone_id} is IDLE. Assigning start phase: {self.current_mission.start_phase}")
                await self.assign_phase(self.current_mission.start_phase, drone_id)
            
        except KeyError as e:
            print(f"[FleetCoordinator] Malformed telemetry message: {e}")

    async def _handle_detections(self, topic: str, payload: Dict[str, Any]):
        """
        Handles incoming detections. Implements Hub-Mediated Consensus.
        (From Spec Appendix A.2)
        """
        try:
            # --- THIS IS THE FIX ---
            # 1. Extract data from payload
            detection_payload = payload['detection']
            track_id = detection_payload['track_id']
            drone_id = payload['drone_id']

            world_pos_data = detection_payload.get("world_position")
            if not world_pos_data:
                print(f"[FleetCoordinator] Ignoring detection {track_id} (no world_position).")
                return
            
            # Re-create the LocalPosition object
            world_pos = LocalPosition(
                x=world_pos_data['x'], 
                y=world_pos_data['y'], 
                z=world_pos_data['z']
            )

            # 2. Check if we are already tracking this target
            if track_id not in self.tracked_targets:
                # This is a new potential target
                new_target = TrackedTarget(
                    track_id=track_id,
                    class_label=detection_payload['class_label'],
                    first_seen=detection_payload['timestamp'],
                    last_seen=detection_payload['timestamp']
                )
                new_target.add_detection(world_pos, drone_id, detection_payload['timestamp'])
                self.tracked_targets[track_id] = new_target
                print(f"[FleetCoordinator] New POTENTIAL target registered: {track_id}")
            
            else:
                # This is an update to an existing target
                target = self.tracked_targets[track_id]
                was_potential = target.status == TargetStatus.POTENTIAL
                
                target.add_detection(world_pos, drone_id, detection_payload['timestamp'])
                
                print(f"[FleetCoordinator] Updated existing target {track_id} (sources: {len(target.supporting_drones)})")

                # --- Hub-Mediated Consensus Logic ---
                if was_potential and target.status == TargetStatus.CONFIRMED:
                    print(f"[FleetCoordinator] CONSENSUS: Target {track_id} is now CONFIRMED.")
                    
                    # Only assign one target as "the" confirmed target for now
                    if not self.confirmed_target_id:
                        self.confirmed_target_id = track_id
                    
                    # Broadcast the confirmed target to all drones
                    if self.comms.is_connected():
                        await self.comms.publish("fleet/target_confirmed", {
                            "track_id": target.track_id,
                            "position": asdict(target.latest_position), # Send as dict
                            "confidence": 1.0,
                            "supporting_drones": list(target.supporting_drones)
                        })
                    
                    # TODO: Assign a delivery task to an idle payload drone
                    # await self.assign_delivery_task(target)
            # --- END OF FIX ---

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
        
        asyncio.create_task(self.comms.subscribe("fleet/telemetry/+", self._handle_telemetry))
        asyncio.create_task(self.comms.subscribe("fleet/detections/+", self._handle_detections))
        asyncio.create_task(self.comms.subscribe_lwt(self._handle_drone_lwt))

    async def load_and_start_mission(self, mission_filename: str):
        """Loads a mission JSON and commands drones to start."""
        print(f"[FleetCoordinator] Received request to load mission: {mission_filename}")
        try:
            if not mission_filename or ".." in mission_filename:
                raise ValueError("Invalid mission filename.")
                
            mission_path = (self.missions_dir / mission_filename).resolve()
            
            if self.missions_dir not in mission_path.parents:
                raise SecurityException(
                    f"Path Traversal Denied: {mission_filename} resolves outside missions directory."
                )
            if not mission_path.is_file():
                raise FileNotFoundError(f"Mission file not found at {mission_path}")
            
            # Get all unique, valid roles defined in the fleet config
            valid_roles = list(set(
                d.get('role') for d in self.fleet_config.values() if d.get('role')
            ))

            mission_dict = load_mission_file(mission_path) 
            self.current_mission = parse_mission_flow(mission_dict, valid_roles)
            
        except (ValueError, FileNotFoundError, SecurityException, MissionParseError) as e:
            print(f"[FleetCoordinator] FAILED to load mission: {e}")
            raise 
        except Exception as e:
            print(f"[FleetCoordinator] FAILED to load mission (unexpected error): {e}")
            raise 

        print(f"[FleetCoordinator] Mission '{self.current_mission.mission_id}' loaded.")
        print(f"[FleetCoordinator] Hub is now WAITING for drones to connect and report IDLE.")
        
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
            
        drones_to_command: List[DroneState] = []
        if target_drone_id:
            if target_drone_id in self.fleet_state:
                drones_to_command = [self.fleet_state[target_drone_id]]
            else:
                print(f"[FleetCoordinator] Cannot assign phase: Target drone {target_drone_id} not in fleet state.")
                return
        else:
            drones_to_command = list(self.fleet_state.values())
            
        print(f"[FleetCoordinator] Assigning phase '{phase_name}' to {len(drones_to_command)} drone(s).")
        
        for drone_state in drones_to_command:
            drone_id = drone_state.drone_id
            drone_role = drone_state.role
            
            task = phase.tasks.get(drone_role)
            
            if task:
                print(f"  -> Sending task '{task.action}' to {drone_id} (Role: {drone_role})")
                command = {
                    "action": "EXECUTE_PHASE",
                    "phase": phase_name,
                    "task": asdict(task)
                }
                if self.comms.is_connected():
                    await self.comms.publish(f"fleet/commands/{drone_id}", command)
            else:
                print(f"  -> No task for {drone_id} (Role: {drone_role}) in this phase. Sending IGNORE.")
                command = {
                    "action": "EXECUTE_PHASE",
                    "phase": phase_name,
                    "task": {"action": "IGNORE"} 
                }
                if self.comms.is_connected():
                    await self.comms.publish(f"fleet/commands/{drone_id}", command)

    async def get_fleet_snapshot(self) -> Dict[str, Any]:
        """Gets a serializable snapshot of the fleet for the GCS."""
        
        # --- THIS IS THE FIX ---
        # Get the confirmed target object, if one exists
        confirmed_target_obj = None
        if self.confirmed_target_id and self.confirmed_target_id in self.tracked_targets:
            confirmed_target_obj = asdict(self.tracked_targets[self.confirmed_target_id])
        
        return {
            "mission_id": self.current_mission.mission_id if self.current_mission else "NONE",
            
            # Send the main confirmed target
            "confirmed_target": confirmed_target_obj,
            
            # Send all drones
            "drones": [asdict(state) for state in self.fleet_state.values()],
            
            # Send all targets (potential, confirmed, etc.)
            "targets": [asdict(target) for target in self.tracked_targets.values()]
        }
        # --- END OF FIX ---

class SecurityException(Exception):
    """Custom exception for security violations like path traversal."""
    pass