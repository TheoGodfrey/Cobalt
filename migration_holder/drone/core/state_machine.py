"""
Formal mission state machine using the 'transitions' library.
(Refactored for COBALT P2P Roles)

This file is based on the original `state_machine.py` from the `drone-mob` repository

and has been significantly modified to implement the P2P roles
described in the "Cobalt drone" document.
"""
from enum import Enum
from transitions.extensions.asyncio import AsyncMachine
from .comms import MqttClient

class MissionPhase(Enum):
    """
    Mission phases, which now directly map to the
    P2P Roles defined in the COBALT document.
    """
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    
    # --- NEW: Explicit P2P Roles from Cobalt Doc ---
    ROLE_SEARCH_PRIMARY = "ROLE_SEARCH_PRIMARY"         # Scout in MOB
    ROLE_SEARCH_DELIVER = "ROLE_SEARCH_DELIVER"         # Payload in MOB (becomes STANDBY)
    ROLE_SEARCH_ASSIST = "ROLE_SEARCH_ASSIST"           # Utility in MOB
    ROLE_EMERGENCY_EYES = "ROLE_EMERGENCY_EYES"         # Scout in Gen-Emerg
    ROLE_EMERGENCY_STANDBY = "ROLE_EMERGENCY_STANDBY"   # Payload in Gen-Emerg (or MOB)
    ROLE_EMERGENCY_ASSIST = "ROLE_EMERGENCY_ASSIST"     # Utility in Gen-Emerg
    ROLE_UTILITY_TASK = "ROLE_UTILITY_TASK"             # Utility/Scout in Patrol
    # ------------------------------------------------
    
    TARGET_PENDING_CONFIRMATION = "TARGET_PENDING_CONFIRMATION"
    TARGET_CONFIRMED = "TARGET_CONFIRMED"
    DELIVERING = "DELIVERING"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    COMPLETED = "COMPLETED"
    EMERGENCY = "EMERGENCY"
    LOCAL_OPERATOR_CONTROL = "LOCAL_OPERATOR_CONTROL"
    
    # --- DEPRECATED (Replaced by roles) ---
    # SEARCHING = "SEARCHING"
    # STANDBY = "STANDBY"
    # PATROLLING = "PATROLLING"
    # OVERWATCH = "OVERWATCH"


class MissionStateMachine:
    """
    Manages state transitions for a single drone in the P2P swarm.
    """
    
    def __init__(self, model, mqtt_client: MqttClient):
        self.model = model
        self.mqtt = mqtt_client
        
        states = [e for e in MissionPhase]

        self.machine = AsyncMachine(
            model=model, 
            states=states, 
            initial=MissionPhase.IDLE,
            after_state_change='_on_state_change'
        )
        
        # --- Simplified Triggers (callbacks in MissionController handle role logic) ---
        
        # --- Standard Mission Start ---
        # These triggers are called by the _p2p_event_listener
        self.machine.add_transition('start_mission', [MissionPhase.IDLE, MissionPhase.ROLE_UTILITY_TASK], MissionPhase.PREFLIGHT, after='_run_preflight')
        self.machine.add_transition('start_standby_mission', [MissionPhase.IDLE, MissionPhase.ROLE_UTILITY_TASK], MissionPhase.PREFLIGHT, after='_run_preflight')
        self.machine.add_transition('start_patrol_mission', MissionPhase.IDLE, MissionPhase.PREFLIGHT, after='_run_preflight')
        self.machine.add_transition('start_overwatch_mission', [MissionPhase.IDLE, MissionPhase.ROLE_UTILITY_TASK], MissionPhase.PREFLIGHT, after='_run_preflight')

        self.machine.add_transition('preflight_success', MissionPhase.PREFLIGHT, MissionPhase.TAKEOFF, after='_run_takeoff')
        
        # --- Post-Takeoff Role Assumption ---
        # The callback in MissionController (_run_takeoff) determines the *correct* altitude
        # based on self.current_mission_type.
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_SEARCH_PRIMARY, 
            after='_run_search_step', 
            conditions=['_is_mob_search', '_is_scout'] # Custom conditions
        )
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_SEARCH_ASSIST, 
            after='_run_search_step', 
            conditions=['_is_mob_search', '_is_payload']
        )

        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_SEARCH_ASSIST, 
            after='_run_search_step', 
            conditions=['_is_mob_search', '_is_utility']
        )
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_EMERGENCY_STANDBY, 
            after='_run_standby', 
            conditions=['_is_standby_mission']
        )
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_UTILITY_TASK, 
            after='_run_patrol', 
            conditions=['_is_patrol_mission']
        )
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_EMERGency_EYES, 
            after='_run_overwatch', 
            conditions=['_is_overwatch_mission']
        )
        # Handle Gen-Emerg assist role
        self.machine.add_transition(
            'takeoff_success', 
            MissionPhase.TAKEOFF, 
            MissionPhase.ROLE_EMERGENCY_ASSIST, 
            after='_run_overwatch', # Assist role also does overwatch
            conditions=['_is_overwatch_mission', '_is_utility']
        )



        

        # --- Target Sighting Logic ---
        self.machine.add_transition(
            'target_sighted', 
            [MissionPhase.ROLE_SEARCH_PRIMARY, MissionPhase.ROLE_SEARCH_ASSIST], 
            MissionPhase.TARGET_PENDING_CONFIRMATION,
            after='_request_operator_confirmation'
        )
        self.machine.add_transition(
            'reject_target', 
            MissionPhase.TARGET_PENDING_CONFIRMATION, 
            MissionPhase.ROLE_SEARCH_PRIMARY, # Go back to searching
            after='_handle_rejection',
            conditions=['_is_scout']
        )
        self.machine.add_transition(
            'reject_target', 
            MissionPhase.TARGET_PENDING_CONFIRMATION, 
            MissionPhase.ROLE_SEARCH_ASSIST, # Go back to searching
            after='_handle_rejection',
            conditions=['_is_utility']
        )
        self.machine.add_transition(
            'reject_target', 
            MissionPhase.TARGET_PENDING_CONFIRMATION, 
            MissionPhase.ROLE_SEARCH_ASSIST, # Go back to searching
            after='_handle_rejection',
            conditions=['_is_payload']
        )
        self.machine.add_transition(
            'confirm_target', 
            MissionPhase.TARGET_PENDING_CONFIRMATION, 
            MissionPhase.TARGET_CONFIRMED,
            after='_request_delivery' # This now broadcasts the P2P event
        )
        
        # --- Payload Drone Logic ---
        self.machine.add_transition(
            'start_delivery_mission',
            [MissionPhase.IDLE, MissionPhase.ROLE_EMERGENCY_STANDBY],
            MissionPhase.PREFLIGHT, # Always preflight before delivery
            after='_run_preflight'
        )
        self.machine.add_transition(
            'takeoff_success',
            MissionPhase.TAKEOFF,
            MissionPhase.DELIVERING,
            after='_run_payload_delivery',
            conditions=['_is_delivery_mission']
        )

        # --- Common End-of-Mission Paths ---
        self.machine.add_transition('search_complete_negative', [MissionPhase.ROLE_SEARCH_PRIMARY, MissionPhase.ROLE_SEARCH_ASSIST], MissionPhase.RETURNING, after='_run_return_to_home')
        self.machine.add_transition('delivery_request_sent', MissionPhase.TARGET_CONFIRMED, MissionPhase.RETURNING, after='_run_return_to_home')
        self.machine.add_transition('delivery_complete', MissionPhase.DELIVERING, MissionPhase.RETURNING, after='_run_return_to_home')
        self.machine.add_transition('patrol_complete', MissionPhase.ROLE_UTILITY_TASK, MissionPhase.RETURNING, after='_run_return_to_home')
        self.machine.add_transition('patrol_battery_low', MissionPhase.ROLE_UTILITY_TASK, MissionPhase.RETURNING, after='_run_return_to_home')
        self.machine.add_transition('overwatch_complete', [MissionPhase.ROLE_EMERGENCY_EYES, MissionPhase.ROLE_EMERGENCY_ASSIST], MissionPhase.RETURNING, after='_run_return_to_home')
        
        self.machine.add_transition('arrived_home', MissionPhase.RETURNING, MissionPhase.LANDING, after='_run_land')
        self.machine.add_transition('land_complete', MissionPhase.LANDING, MissionPhase.COMPLETED, after='_log_mission_summary')

        # --- Emergency & Operator Takeover ---
        self.machine.add_transition('trigger_emergency', '*', MissionPhase.EMERGENCY, after='_run_emergency_land')
        self.machine.add_transition('local_operator_takeover', '*', MissionPhase.LOCAL_OPERATOR_CONTROL, after='_run_local_operator_takeover')
        self.machine.add_transition('local_operator_release', MissionPhase.LOCAL_OPERATOR_CONTROL, MissionPhase.RETURNING, after='_run_local_operator_release')
        
        # --- Final cleanup ---
        self.machine.add_transition('mission_finished', MissionPhase.COMPLETED, MissionPhase.IDLE)
        self.machine.add_transition('reset_from_emergency', MissionPhase.EMERGENCY, MissionPhase.IDLE)


    async def _on_state_change(self, event):
        """Log all state changes and publish them to MQTT for the Hub/GCS."""
        new_state = str(event.state.name)
        self.model.logger.log(f"State changed to: {new_state}", "info")
        await self.mqtt.publish(
            f"fleet/state/{self.model.drone.id}",
            {"state": new_state, "drone_id": self.model.drone.id, "role": self.model.role}
        )
    
    # --- Helper methods for conditions ---
    def _is_scout(self): return self.model.role == 'scout'
    def _is_payload(self): return self.model.role == 'payload'
    def _is_utility(self): return self.model.role == 'utility'
    
    def _is_mob_search(self): return self.model.current_mission_type == 'MOB_SEARCH'
    def _is_standby_mission(self): return self.model.current_mission_type == 'STANDBY'
    def _is_patrol_mission(self): return self.model.current_mission_type == 'PATROL'
    def _is_overwatch_mission(self): return self.model.current_mission_type == 'OVERWATCH'
    def _is_delivery_mission(self): return self.model.current_mission_type == 'PAYLOAD_DELIVERY'