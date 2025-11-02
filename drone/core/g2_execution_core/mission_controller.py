"""
Component 3: Mission Controller
Generic flow interpreter - NO mission-specific logic
"""
from ..g1_mission_definition.mission_flow import MissionFlow
from ..g2_execution_core.mission_state import MissionState
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
        
        self.behavior_factory = BehaviorFactory(config)
    
    async def execute_mission(self):
        """Generic mission execution loop"""
        # Start at the defined start phase
        await self.mission_state.transition_to(self.mission_flow.start_phase)
        
        while not self.mission_state.is_complete():
            # Get current phase definition from JSON
            phase = self.mission_flow.get_phase(self.mission_state.current_phase)
            
            # Execute all tasks for this drone's role
            drone_role = self.get_my_role()
            if drone_role in phase.tasks:
                task = phase.tasks[drone_role]
                behavior = self.behavior_factory.create(task)
                await behavior.run()
            
            # Evaluate transitions
            trigger = await self.wait_for_trigger(phase.transitions)
            if trigger:
                next_phase = phase.transitions[trigger][drone_role]
                await self.mission_state.transition_to(next_phase)
    
    async def wait_for_trigger(self, transitions: dict) -> str:
        """Wait for any transition trigger to fire"""
        # Listen to MQTT events, timeouts, behavior completion, etc.
        # This is GENERIC - reads trigger definitions from JSON
        pass