"""
Component 3: Mission Controller
Generic flow interpreter - NO mission-specific logic
"""
import asyncio
from ..g1_mission_definition.mission_flow import MissionFlow
# Import the enum as well to map strings to enum members
from ..g2_execution_core.mission_state import MissionState, MissionStateEnum
from ..g2_execution_core.behaviors import BehaviorFactory

class MissionController:
    """Generic orchestrator that executes any mission from JSON"""
    
    def __init__(self, mission_flow: MissionFlow, flight_controller, 
                 vehicle_state, mqtt, safety_monitor, config):
        self.mission_flow = mission_flow
        self.mission_state = MissionState()
        self.flight_controller = flight_controller
        self.vehicle_state = vehicle_state
        self.mqtt = mqtt
        self.safety_monitor = safety_monitor
        self.config = config
        
        # This assumes BehaviorFactory is implemented in behaviors.py (Bug #1)
        self.behavior_factory = BehaviorFactory(config)
    
    # FIX for Bug #2: Implement get_my_role()
    def get_my_role(self) -> str:
        """
        Determines the drone's role from its client ID.
        This is a simple implementation based on naming conventions.
        e.g., "scout_1" -> "scout", "payload_1" -> "payload"
        """
        # The drone_id is used as the mqtt client_id
        client_id = self.mqtt._client_id 
        
        if client_id.startswith("scout"):
            return "scout"
        if client_id.startswith("payload"):
            return "payload"
        if client_id.startswith("utility"):
            return "utility"
            
        # Fallback role if no convention matches
        print(f"[MissionController] WARNING: Could not determine role from ID '{client_id}'. Defaulting to 'scout'.")
        return "scout"

    async def execute_mission(self):
        """Generic mission execution loop"""
        
        try:
            # Start at the defined start phase
            start_phase_enum = MissionStateEnum[self.mission_flow.start_phase.upper()]
            
            # FIX: Method is .transition(), not .transition_to()
            await self.mission_state.transition(start_phase_enum)
            
            # Check if mission is complete (e.g., if start_phase was RTH)
            while not self.mission_state.current in (MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.ABORTED, MissionStateEnum.IDLE):
                # Get current phase definition from JSON
                # Note: self.mission_state.current.name is e.g., "SEARCHING"
                # We need the lowercase version for the key
                current_phase_key = self.mission_state.current.name.lower()
                phase = self.mission_flow.phases[current_phase_key]
                
                # Execute all tasks for this drone's role
                drone_role = self.get_my_role()
                if drone_role in phase.tasks:
                    task = phase.tasks[drone_role]
                    
                    # FIX: Pass all required dependencies to the factory
                    behavior = self.behavior_factory.create(
                        task, 
                        self.mission_state, 
                        self.flight_controller  # The HAL is passed in as flight_controller
                    )
                    await behavior.run()
                
                # Evaluate transitions
                # Bug #3: wait_for_trigger is unimplemented and will hang
                trigger = await self.wait_for_trigger(phase.transitions)
                
                if trigger:
                    # FIX: Parse "goto:delivery" string to MissionStateEnum.DELIVERING
                    next_phase_str = phase.transitions[trigger][drone_role]
                    next_phase_name = next_phase_str.replace("goto:", "").upper()
                    next_phase_enum = MissionStateEnum[next_phase_name]
                    
                    # FIX: Method is .transition(), not .transition_to()
                    await self.mission_state.transition(next_phase_enum)
                else:
                    # If wait_for_trigger returns None (e.g., not implemented)
                    print("[MissionController] No trigger returned. Assuming mission is stuck.")
                    break
                    
        except KeyError as e:
            print(f"[MissionController] CRITICAL ERROR: Phase mismatch or invalid key: {e}")
            print(f"  Check if '{current_phase_key}' or (if defined) '{next_phase_name}' exists in mission file and MissionStateEnum.")
            # FIX: Method is .transition(), not .transition_to()
            await self.mission_state.transition(MissionStateEnum.ABORTED)
        except Exception as e:
            print(f"[MissionController] CRITICAL ERROR during execution: {e}")
            # FIX: Method is .transition(), not .transition_to()
            await self.mission_state.transition(MissionStateEnum.ABORTED)
    
    async def wait_for_trigger(self, transitions: dict) -> str:
        """
        Wait for any transition trigger to fire
        
        BUG #3: This logic is unimplemented
        """
        # Listen to MQTT events, timeouts, behavior completion, etc.
        # This is GENERIC - reads trigger definitions from JSON
        print("[MissionController] WARNING: wait_for_trigger() is not implemented (Bug #3). Mission will hang.")
        await asyncio.sleep(3600) # Simulate a long wait to show it's stuck
        return None # Return None to stop loop