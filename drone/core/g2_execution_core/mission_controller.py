"""
Component 3: Mission Controller
Generic flow interpreter - NO mission-specific logic
"""
import asyncio
from typing import Optional, Dict, Set, Any
from ..g1_mission_definition.mission_flow import MissionFlow
# Import the enum as well to map strings to enum members
from ..g2_execution_core.mission_state import (
    MissionState, MissionStateEnum, 
    MISSION_PHASE_TO_ENUM, MISSION_ENUM_TO_PHASE # <-- FIX for Bug #11
)
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
        
        # FIX for Bug #5: Track triggered events
        self._triggered_events: Set[str] = set()
        self._event_lock = asyncio.Lock()
        
        # Track state change listeners for cleanup
        self._state_listeners = []
    
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
        
        current_phase_key = "None" # For error logging
        next_phase_name = "None" # For error logging
        
        try:
            # Start at the defined start phase
            
            # --- FIX for Bug #11 ---
            start_phase_key = self.mission_flow.start_phase
            if start_phase_key not in MISSION_PHASE_TO_ENUM:
                raise ValueError(f"Start phase '{start_phase_key}' in mission file is not a valid phase name.")
            start_phase_enum = MISSION_PHASE_TO_ENUM[start_phase_key]
            # --- End of Fix ---
            
            # FIX: Method is .transition(), not .transition_to()
            self.mission_state.transition(start_phase_enum)
            
            # Check if mission is complete (e.g., if start_phase was RTH)
            while not self.mission_state.current in (MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.ABORTED, MissionStateEnum.IDLE):
                # Get current phase definition from JSON
                
                # --- FIX for Bug #11 (This is the line 73 issue) ---
                if self.mission_state.current not in MISSION_ENUM_TO_PHASE:
                    raise ValueError(f"Current state '{self.mission_state.current.name}' has no mapping to a phase key.")
                current_phase_key = MISSION_ENUM_TO_PHASE[self.mission_state.current]
                # --- End of Fix ---
                
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
                    await behavior.start()
                
                # Evaluate transitions
                # Bug #5: wait_for_trigger is unimplemented and will hang
                trigger = await self.wait_for_trigger(phase.transitions)
                
                if trigger:
                    # FIX: Parse "goto:delivery" string to MissionStateEnum.DELIVERING
                    next_phase_str = phase.transitions[trigger][drone_role]
                    
                    # --- FIX for Bug #11 ---
                    next_phase_key = next_phase_str.replace("goto:", "")
                    if next_phase_key not in MISSION_PHASE_TO_ENUM:
                        raise ValueError(f"Transition 'goto' phase '{next_phase_key}' is not a valid phase name.")
                    next_phase_enum = MISSION_PHASE_TO_ENUM[next_phase_key]
                    next_phase_name = next_phase_key # For logging
                    # --- End of Fix ---
                    
                    # FIX: Method is .transition(), not .transition_to()
                    self.mission_state.transition(next_phase_enum)
                else:
                    # If wait_for_trigger returns None (e.g., not implemented)
                    print("[MissionController] No trigger returned. Assuming mission is stuck.")
                    break
                    
        except KeyError as e:
            print(f"[MissionController] CRITICAL ERROR: Phase mismatch or invalid key: {e}")
            print(f"  Check if '{current_phase_key}' or (if defined) '{next_phase_name}' exists in mission file and MissionStateEnum.")
            # FIX: Method is .transition(), not .transition_to()
            self.mission_state.transition(MissionStateEnum.ABORTED)
        except Exception as e:
            print(f"[MissionController] CRITICAL ERROR during execution: {e}")
            # FIX: Method is .transition(), not .transition_to()
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            # Cleanup listeners
            await self._cleanup_listeners()
    
    # ====================================================================================
    # FIX for Bug #5: Implement wait_for_trigger
    # ====================================================================================
    
    async def wait_for_trigger(self, transitions: Dict[str, Dict[str, str]], timeout: float = 3600.0) -> Optional[str]:
        """
        Wait for any transition trigger to fire.
        
        This is a GENERIC implementation that reads trigger definitions from the mission JSON
        and monitors for:
        1. MQTT events (on_event:*)
        2. Mission state changes (on_state:*)
        3. Vehicle state changes (on_vehicle_state:*)
        4. Timeouts (on_timeout:*)
        
        Args:
            transitions: Dict mapping trigger strings to destination phases
            timeout: Maximum time to wait in seconds (default: 1 hour)
            
        Returns:
            The trigger string that fired (e.g., "on_event:target_found")
            or None if timeout occurs
            
        Example transition dict:
        {
            "on_event:target_found": {"scout": "goto:delivery", "payload": "goto:delivery"},
            "on_state:ABORTED": {"scout": "goto:idle", "payload": "goto:idle"},
            "on_timeout:300": {"scout": "goto:returning", "payload": "goto:returning"}
        }
        """
        
        if not transitions:
            print("[MissionController] No transitions defined for this phase. Phase will complete immediately.")
            return None
        
        print(f"[MissionController] Waiting for one of {len(transitions)} possible triggers...")
        
        # Clear any previously triggered events
        async with self._event_lock:
            self._triggered_events.clear()
        
        # Parse the transitions and set up listeners
        event_triggers = []
        state_triggers = []
        timeout_triggers = []
        
        for trigger_key in transitions.keys():
            if trigger_key.startswith("on_event:"):
                event_name = trigger_key.split(":", 1)[1]
                event_triggers.append((trigger_key, event_name))
                
            elif trigger_key.startswith("on_state:"):
                state_name = trigger_key.split(":", 1)[1]
                state_triggers.append((trigger_key, state_name))
                
            elif trigger_key.startswith("on_timeout:"):
                timeout_seconds = float(trigger_key.split(":", 1)[1])
                timeout_triggers.append((trigger_key, timeout_seconds))
        
        # Create monitoring tasks
        tasks = []
        
        # Monitor MQTT events
        if event_triggers:
            event_task = asyncio.create_task(
                self._monitor_events(event_triggers),
                name="monitor_events"
            )
            tasks.append(event_task)
        
        # Monitor mission state changes
# Monitor mission state changes
        if state_triggers:
            state_task = asyncio.create_task(
                self._monitor_state_changes(state_triggers, self.mission_state), # <-- Pass self.mission_state
                name="monitor_states"
            )
            tasks.append(state_task)
        
        # Set up timeout triggers
        for trigger_key, timeout_seconds in timeout_triggers:
            timeout_task = asyncio.create_task(
                self._wait_timeout(trigger_key, timeout_seconds),
                name=f"timeout_{timeout_seconds}"
            )
            tasks.append(timeout_task)
        
        # Add a master timeout
        master_timeout_task = asyncio.create_task(
            asyncio.sleep(timeout),
            name="master_timeout"
        )
        tasks.append(master_timeout_task)
        
        try:
            # Wait for the first task to complete
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            
            # Cancel all pending tasks
            for task in pending:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            
            # Get the result from the completed task
            completed_task = done.pop()
            
            if completed_task == master_timeout_task:
                print(f"[MissionController] Master timeout ({timeout}s) reached. No trigger fired.")
                return None
            
            # Get the trigger that fired
            triggered = await completed_task
            print(f"[MissionController] Trigger fired: {triggered}")
            return triggered
            
        except Exception as e:
            print(f"[MissionController] Error in wait_for_trigger: {e}")
            # Cancel all tasks on error
            for task in tasks:
                task.cancel()
            return None
    
    async def _monitor_events(self, event_triggers: list) -> str:
        """
        Monitors MQTT for fleet-wide events.
        
        Args:
            event_triggers: List of (trigger_key, event_name) tuples
            
        Returns:
            The trigger_key that fired
        """
        print(f"[MissionController] Monitoring for events: {[e[1] for e in event_triggers]}")
        
        # Subscribe to fleet-wide event topics
        event_names = [event_name for _, event_name in event_triggers]
        
        async def event_callback(topic: str, payload: Dict[str, Any]):
            """Called when an event message arrives."""
            event_type = payload.get("event_type")
            
            # Check if this event matches any of our triggers
            for trigger_key, event_name in event_triggers:
                if event_type == event_name:
                    async with self._event_lock:
                        self._triggered_events.add(trigger_key)
                    print(f"[MissionController] Event detected: {event_name}")
        
        # Subscribe to the fleet events topic
        await self.mqtt.subscribe("fleet/events/+", event_callback)
        
        # Poll until an event is triggered
        while True:
            async with self._event_lock:
                for trigger_key, _ in event_triggers:
                    if trigger_key in self._triggered_events:
                        return trigger_key
            await asyncio.sleep(0.1)
    
    async def _monitor_state_changes(self, state_triggers: list, mission_state: MissionState) -> str:
        """
        Monitors MissionState for state changes.
        
        Args:
            state_triggers: List of (trigger_key, state_name) tuples
            mission_state: The current mission state object
            
        Returns:
            The trigger_key that fired
        """
        print(f"[MissionController] Monitoring for state changes: {[s[1] for s in state_triggers]}")

        # --- FIX: First, check if the state is ALREADY correct ---
        current_state_name = mission_state.current.name.upper()
        for trigger_key, expected_state in state_triggers:
            if current_state_name == expected_state.upper():
                print(f"[MissionController] State trigger '{trigger_key}' is already active.")
                return trigger_key
        # --- End of FIX ---
        
        # Create a future that resolves when the state changes
        state_future = asyncio.Future()
        
        def state_listener(new_state: MissionStateEnum):
            """Called when mission state changes."""
            state_name = new_state.name.upper()
            
            for trigger_key, expected_state in state_triggers:
                if state_name == expected_state.upper():
                    if not state_future.done():
                        state_future.set_result(trigger_key)
        
        # Register the listener
        self.mission_state.add_listener(state_listener)
        self._state_listeners.append(state_listener)
        
        try:
            # Wait for the state change
            return await state_future
        finally:
            # Cleanup
            self.mission_state.remove_listener(state_listener)
            if state_listener in self._state_listeners:
                self._state_listeners.remove(state_listener)
                
    async def _wait_timeout(self, trigger_key: str, timeout_seconds: float) -> str:
        """
        Waits for a specified timeout and returns the trigger.
        
        Args:
            trigger_key: The trigger key (e.g., "on_timeout:300")
            timeout_seconds: Time to wait in seconds
            
        Returns:
            The trigger_key after timeout expires
        """
        print(f"[MissionController] Timeout trigger set for {timeout_seconds}s")
        await asyncio.sleep(timeout_seconds)
        print(f"[MissionController] Timeout trigger fired: {trigger_key}")
        return trigger_key
    
    async def _cleanup_listeners(self):
        """Remove all registered listeners."""
        for listener in self._state_listeners:
            self.mission_state.remove_listener(listener)
        self._state_listeners.clear()
    
    # ====================================================================================
    # Helper methods for external trigger injection (e.g., from Hub or GCS)
    # ====================================================================================
    
    async def trigger_event(self, event_name: str):
        """
        Manually trigger an event (useful for testing or external commands).
        
        Args:
            event_name: The name of the event to trigger
        """
        trigger_key = f"on_event:{event_name}"
        async with self._event_lock:
            self._triggered_events.add(trigger_key)
        print(f"[MissionController] Event manually triggered: {event_name}")