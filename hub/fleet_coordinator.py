"""
Fleet Coordinator
This is the "brain" of the hub. It acts as the master
Mission Controller (Component 3) for the entire fleet.

It listens to all drones, manages fleet-wide state,
and issues commands based on the master mission plan.
"""

import asyncio
# FIX for Bug #10: Import asdict
from dataclasses import dataclass, field, asdict
from time import time
from typing import Dict, Any, Optional, Set

# We need the communication layer
from drone.core.cross_cutting.communication import MqttClient

# We need the mission definition files (G1) to load
from drone.core.g1_mission_definition.loader import load_mission_file
# FIX for Bug #9: Function is parse_mission_flow, not parse_mission
from drone.core.g1_mission_definition.parser import parse_mission_flow
from drone.core.g1_mission_definition.mission_flow import MissionFlow

# Placeholder for a drone's state as tracked by the hub
@dataclass
class DroneState:
    drone_id: str
    last_telemetry: Dict[str, Any] = field(default_factory=dict)
    mission_state: str = "IDLE"
    last_heartbeat: float = 0.0

class FleetCoordinator:
    """
    Manages the entire fleet of drones, assigns tasks,
    and acts as the central Mission Controller.
    """
    
    def __init__(self, comms: MqttClient):
        self.comms = comms
        self.fleet_state: Dict[str, DroneState] = {}
        self.current_mission: Optional[MissionFlow] = None
        self.active_detections: Dict[str, Any] = {}
        self.confirmed_target: Optional[Dict[str, Any]] = None
        print("[FleetCoordinator] Initialized.")

    async def _handle_telemetry(self, msg: Dict[str, Any]):
        """Callback for processing telemetry messages."""
        try:
            drone_id = msg['drone_id']
            if drone_id not in self.fleet_state:
                self.fleet_state[drone_id] = DroneState(drone_id=drone_id)
            
            state = self.fleet_state[drone_id]
            state.last_telemetry = msg['telemetry']
            state.mission_state = msg['mission_state']
            state.last_heartbeat = time.monotonic()
            
            # print(f"[FleetCoordinator] Telemetry from {drone_id}: {state.mission_state}")
        except KeyError as e:
            print(f"[FleetCoordinator] Malformed telemetry message: {e}")

    async def _handle_detections(self, msg: Dict[str, Any]):
        """
        Handles incoming detections. Implements Hub-Mediated Consensus.
        (From Spec Appendix A.2)
        """
        try:
            drone_id = msg['drone_id']
            detection = msg['detection']
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
                await self.comms.publish("fleet/target_confirmed", {
                    "position": self.confirmed_target['position'],
                    "confidence": 1.0,
                    "supporting_drones": list(self.active_detections.keys())
                })
                
                # Tell drones to switch to delivery phase (simplified)
                await self.assign_phase("delivery")

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

    async def load_and_start_mission(self, mission_file_path: str):
        """Loads a mission JSON and commands drones to start."""
        print(f"[FleetCoordinator] Loading mission: {mission_file_path}")
        try:
            mission_dict = load_mission_file(mission_file_path)
            # FIX for Bug #9: Use parse_mission_flow
            self.current_mission = parse_mission_flow(mission_dict)
        except Exception as e:
            print(f"[FleetCoordinator] FAILED to load mission: {e}")
            return

        print(f"[FleetCoordinator] Mission '{self.current_mission.mission_id}' loaded.")
        
        # Tell drones to start the first phase
        await self.assign_phase(self.current_mission.start_phase)

    async def assign_phase(self, phase_name: str):
        """Commands all drones to execute a specific phase."""
        if not self.current_mission:
            print("[FleetCoordinator] No mission loaded, cannot assign phase.")
            return

        phase = self.current_mission.phases.get(phase_name)
        if not phase:
            print(f"[FleetCoordinator] Unknown phase '{phase_name}'")
            return
            
        print(f"[FleetCoordinator] Assigning phase '{phase_name}' to all drones.")
        
        # This is the "orchestration" step
        # A real system would assign tasks based on role (scout, payload)
        for drone_id in self.fleet_state:
            task = phase.tasks.get("scout") # Simplified: everyone is a scout
            if task:
                command = {
                    "action": "EXECUTE_PHASE",
                    "phase": phase_name,
                    # FIX for Bug #10: Use asdict() instead of model_dump()
                    "task": asdict(task) # Send task config
                }
                await self.comms.publish(f"fleet/commands/{drone_id}", command)

    async def get_fleet_snapshot(self) -> Dict[str, Any]:
        """Gets a serializable snapshot of the fleet for the GCS."""
        return {
            "mission_id": self.current_mission.mission_id if self.current_mission else "NONE",
            "confirmed_target": self.confirmed_target,
            # FIX for Bug #10: Use asdict() instead of model_dump()
            "drones": [asdict(state) for state in self.fleet_state.values()]
        }