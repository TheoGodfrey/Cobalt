"""
Component 3: Mission Controller
Generic flow interpreter - NO mission-specific logic
"""
import asyncio
from typing import Optional, Dict, Set, Any, List, Coroutine

from ..g1_mission_definition.mission_flow import MissionFlow
# Import the enum as well to map strings to enum members
from ..g2_execution_core.mission_state import (
    MissionState, MissionStateEnum,
    MISSION_PHASE_TO_ENUM, MISSION_ENUM_TO_PHASE # <-- FIX for Bug #11
)
from .behaviors import BehaviorFactory, BaseBehavior
from ..cross_cutting.communication import MqttClient # <-- NEW
from ..g4_platform_interface.vehicle_state import VehicleState # <-- NEW
from ..g1_mission_definition.phase import Task # <-- Keep this for dynamic tasking

class MissionController:
    """Generic orchestrator that executes any mission from JSON"""

    def __init__(self, mission_flow: MissionFlow, flight_controller,
                 vehicle_state: VehicleState, mqtt: MqttClient, safety_monitor, config, # <-- MODIFIED
                 hardware_list: List[str], drone_role: str,
                 mission_active_event: asyncio.Event): # <-- MODIFIED
        self.mission_flow = mission_flow
        self.mission_state = MissionState()
        self.flight_controller = flight_controller
        self.vehicle_state = vehicle_state
        self.mqtt = mqtt
        self.safety_monitor = safety_monitor
        self.config = config
        self.hardware_list = hardware_list # NEW: Store hardware_list
        self.drone_role = drone_role # <-- MODIFIED: Store the role
        self.mission_active_event = mission_active_event # <-- ADDED

        # This assumes BehaviorFactory is implemented in behaviors.py (Bug #1)
        # NEW: Pass hardware_list to factory
        self.behavior_factory = BehaviorFactory(config, self.hardware_list)

        # --- NEW: Track current behavior and if it's hub-driven ---
        self.current_behavior: Optional[BaseBehavior] = None
        self.is_hub_driven = False # Start as autonomous
        # --- End NEW ---

        # --- NEW: Event-driven trigger logic ---
        # Stores the valid triggers for the *current* phase
        self._current_phase_triggers: Set[str] = set()
        # An event that the main loop waits on, set by *any* valid trigger
        self._trigger_event_fired = asyncio.Event()
        self._fired_trigger: Optional[str] = None
        self._event_lock = asyncio.Lock() # Lock for _current_phase_triggers

        # --- NEW: Concurrency lock for transitions ---
        self._is_transitioning = False
        self._transition_lock = asyncio.Lock()
        # --- End of NEW ---

        # The persistent MQTT listener task
        self._mqtt_listener_task: Optional[asyncio.Task] = None
        # --- End of NEW ---

        # Track state change listeners for cleanup
        self._state_listeners = []

    async def execute_mission(self):
        """Generic mission execution loop"""

        current_phase_key = "None" # For error logging
        next_phase_name = "None" # For error logging

        # --- NEW: Start the persistent MQTT listener ---
        self._mqtt_listener_task = asyncio.create_task(self._persistent_mqtt_listener())
        # --- End of NEW ---

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
            while not self.mission_state.current in (MissionStateEnum.MISSION_COMPLETE, MissionStateEnum.ABORTED, MissionStateEnum.IDLE) and self.mission_active_event.is_set(): # <-- MODIFIED

                # --- NEW: Start of atomic transition section ---
                # We acquire the lock here to ensure the *entire* transition
                # from one phase to the next is atomic.
                async with self._transition_lock:
                    self._is_transitioning = True # Set flag to block new triggers

                    # NEW: Store the state before behavior starts for comparison later
                    state_at_start_of_phase = self.mission_state.current

                    # 1. Get current phase definition from JSON
                    if self.mission_state.current not in MISSION_ENUM_TO_PHASE:
                        raise ValueError(f"Current state '{self.mission_state.current.name}' has no mapping to a phase key.")
                    current_phase_key = MISSION_ENUM_TO_PHASE[self.mission_state.current]

                    phase = self.mission_flow.phases[current_phase_key]

                    # 2. Execute all tasks for this drone's role
                    drone_role = self.drone_role # <-- MODIFIED
                    if drone_role in phase.tasks:
                        # --- MODIFIED: Use the local mission task ---
                        # This is the "default" autonomous behavior
                        task = phase.tasks[drone_role]

                        # --- MODIFIED: Pass 7 arguments correctly ---
                        self.current_behavior = self.behavior_factory.create(
                            task,                   # 1
                            self.mission_state,     # 2
                            self.flight_controller, # 3
                            self.mqtt,              # 4
                            self.mqtt._client_id,   # 5 (drone_id)
                            self.config,            # 6 (config)
                            self.drone_role         # 7 (drone_role)
                        )
                        # --- End of MODIFIED ---
                        await self.current_behavior.start()

                    # 3. Check for transitions
                    transitions = phase.transitions
                    if not transitions:
                        print(f"[MissionController] Phase '{current_phase_key}' has no transitions. Mission will end.")
                        break # No transitions, exit loop

                    # 4. Clear the event from the last phase
                    self._trigger_event_fired.clear()
                    self._fired_trigger = None

                    # 5. Set the valid triggers for this new phase
                    async with self._event_lock:
                        self._current_phase_triggers = set(transitions.keys())
                    print(f"[MissionController] Phase '{current_phase_key}' started. Waiting for triggers: {self._current_phase_triggers}")

                    # 6. Create tasks for phase-specific triggers (state, timeout)
                    phase_specific_tasks = self._create_phase_listeners(transitions)

                    # 7. We are now ready to accept new triggers
                    self.is_hub_driven = False # We are now running local logic
                    self._is_transitioning = False

                # --- End of atomic transition section (lock released) ---

                # 8. Wait for a trigger (global MQTT or phase-specific) OR shutdown
                wait_tasks = [
                    asyncio.create_task(self._trigger_event_fired.wait(), name="wait_for_global_trigger"),
                    asyncio.create_task(self._wait_for_shutdown_signal(), name="wait_for_shutdown")
                ]

                done, pending = await asyncio.wait(
                    wait_tasks + phase_specific_tasks,
                    return_when=asyncio.FIRST_COMPLETED
                )

                # --- NEW: Acquire lock to handle the fired trigger ---
                async with self._transition_lock:
                    self._is_transitioning = True # Block new triggers

                    # 9. A trigger has fired or we are shutting down. Stop behavior.
                    if self.current_behavior:
                        await self.current_behavior.stop()
                        self.current_behavior = None

                    # 10. Cancel all other pending listeners for this phase
                    for task in pending:
                        task.cancel()

                    # 11. Check if shutdown was the trigger
                    if not self.mission_active_event.is_set():
                        print("[MissionController] Shutdown event received, exiting mission loop.")
                        break # Exit the main while loop

                    # --- FIX: Handle Hub command override and reset event for next iteration ---
                    if self._fired_trigger and self._fired_trigger.startswith("hub_command:"):
                        # Hub commands were handled by _execute_hub_task and a new behavior was started.
                        # We clear the event and continue to the next iteration to re-setup listeners.

                        # CRITICAL: Reset the event and flag for the next loop iteration
                        self._trigger_event_fired.clear()
                        self._fired_trigger = None

                        self._is_transitioning = False # Unlocking early
                        continue # Loop again to re-start the (new) behavior
                    # --- End FIX ---

                    # 12. Get the trigger that fired
                    trigger = self._fired_trigger
                    if not trigger:
                        # This could happen if a state/timeout task won the race
                        # but hadn't set self._fired_trigger yet. Find it.
                        for task in done:
                            if task.get_name().startswith("state_") or task.get_name().startswith("timeout_"): # FIX: Use get_name()
                                try:
                                    trigger = await task # Get the result (trigger key)
                                except asyncio.CancelledError:
                                    continue # This task was cancelled, check others
                                if trigger:
                                    break

                    if trigger:
                        # We have a valid trigger, find the next phase
                        next_phase_str = transitions[trigger][self.drone_role] # <-- MODIFIED

                        next_phase_key = next_phase_str.replace("goto:", "")
                        if next_phase_key not in MISSION_PHASE_TO_ENUM:
                            raise ValueError(f"Transition 'goto' phase '{next_phase_key}' is not a valid phase name.")
                        next_phase_enum = MISSION_PHASE_TO_ENUM[next_phase_key]
                        next_phase_name = next_phase_key # For logging

                        self.mission_state.transition(next_phase_enum)
                    else:
                        # CRITICAL FIX for Spurious Wake-Up: Check for behavior success by state change
                        if self.mission_state.current != state_at_start_of_phase:
                             # The behavior succeeded and transitioned the state directly (e.g., SEARCHING -> DELIVERING)
                             print(f"[MissionController] SUCCESS: Behavior completion detected state change: {state_at_start_of_phase.name} -> {self.mission_state.current.name}. Continuing mission flow.")
                             self._is_transitioning = False # Release lock and re-wait
                             continue # Restart the while loop to process the new phase

                        # FIX: Add an explicit check to continue the loop if the state is still active
                        elif self.mission_state.current in (MissionStateEnum.SEARCHING, MissionStateEnum.DELIVERING, MissionStateEnum.RETURNING):
                             print(f"[MissionController] WARNING: Spurious wake-up detected while active ({self.mission_state.current.name}). Re-running phase.")
                             self._is_transitioning = False # Release lock and re-wait
                             continue # Restart the while loop

                        # If we are here, we woke up, but nothing changed, and we are not in an active mission phase.
                        print("[MissionController] No trigger found but loop ended. Aborting.")
                        self.mission_state.transition(MissionStateEnum.ABORTED)
                        break

                    # 13. Mark transition as complete
                    self._is_transitioning = False
                # --- End of trigger handling lock ---

            if not self.mission_active_event.is_set():
                print("[MissionController] Mission externally shut down. Aborting.")
                self.mission_state.transition(MissionStateEnum.ABORTED)

        except KeyError as e:
            print(f"[MissionController] CRITICAL ERROR: Phase mismatch or invalid key: {e}")
            print(f"  Check if '{current_phase_key}' or (if defined) '{next_phase_name}' exists in mission file and MissionStateEnum.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        except asyncio.CancelledError:
            print("[MissionController] Execute mission loop cancelled.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        except Exception as e:
            print(f"[MissionController] CRITICAL ERROR during execution: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            # Cleanup main tasks
            if self.current_behavior:
                await self.current_behavior.stop()
            if self._mqtt_listener_task:
                self._mqtt_listener_task.cancel()
            await self._cleanup_listeners()
            # --- NEW: Signal all other loops to shut down ---
            print("[MissionController] Mission complete. Signaling shutdown.")
            self.mission_active_event.clear()
            # --- End of NEW ---

    # ====================================================================================
    # NEW: Persistent MQTT Listener and Handlers
    # ====================================================================================

    async def _persistent_mqtt_listener(self):
        """
        A single, persistent task that listens for all relevant external MQTT topics.
        """
        try:
            print("[MissionController] Starting persistent MQTT listener...")
            my_id = self.mqtt._client_id

            # Subscribe to Hub commands
            await self.mqtt.subscribe(
                f"fleet/commands/{my_id}",
                self._handle_hub_command
            )
            # Subscribe to Hub-confirmed events
            await self.mqtt.subscribe(
                "fleet/events/+",
                self._handle_fleet_event
            )
            # Subscribe to peer detections
            await self.mqtt.subscribe(
                "fleet/detections/+",
                self._handle_peer_detection
            )

            print("[MissionController] MQTT subscriptions active.")
            # Keep the listener alive until the mission ends
            await self._wait_for_shutdown_signal()

        except asyncio.CancelledError:
            print("[MissionController] Persistent MQTT listener cancelled.")
        except Exception as e:
            print(f"[MissionController] CRITICAL ERROR in MQTT listener: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            self.mission_active_event.clear() # Trigger shutdown

    async def _handle_hub_command(self, topic: str, payload: Dict[str, Any]):
        """Callback for messages on 'fleet/commands/{self.drone_id}'."""
        command = payload.get("command", payload.get("action")) # Allow "action"
        print(f"[MissionController] Received Hub Command: {command}")

        # --- NEW: Dynamic Tasking Logic ---
        if command == "EXECUTE_PHASE":
            await self._execute_hub_task(payload)
            return # Don't process as a simple trigger
        # --- End NEW ---

        trigger_key = f"on_command:{command}"
        await self._check_and_fire_trigger(trigger_key, payload)

    async def _handle_fleet_event(self, topic: str, payload: Dict[str, Any]):
        """Callback for messages on 'fleet/events/+'."""
        event_type = payload.get("event_type", "unknown")
        trigger_key = f"on_event:{event_type}"
        print(f"[MissionController] Received Fleet Event: {event_type}")
        await self._check_and_fire_trigger(trigger_key, payload)

    async def _handle_peer_detection(self, topic: str, payload: Dict[str, Any]):
        """Callback for messages on 'fleet/detections/+'."""
        # Don't fire for our own detections
        if payload.get("drone_id") == self.mqtt._client_id:
            return

        trigger_key = "on_event:peer_detection"
        print(f"[MissionController] Received Peer Detection from {payload.get('drone_id')}")
        await self._check_and_fire_trigger(trigger_key, payload)

    async def _check_and_fire_trigger(self, trigger_key: str, payload: Optional[Dict] = None):
        """
        Central logic to check if a trigger is valid for the current phase
        and fire the main event if it is.
        """
        # --- NEW: Check if we are already transitioning ---
        if self._is_transitioning:
            print(f"[MissionController] Discarding trigger '{trigger_key}' (transition in progress).")
            return
        # --- End of NEW ---

        async with self._event_lock:
            if trigger_key in self._current_phase_triggers:
                if not self._trigger_event_fired.is_set():
                    print(f"[MissionController] TRIGGER FIRED: {trigger_key}")
                    self._fired_trigger = trigger_key
                    self._trigger_event_fired.set()
                    # TODO: Pass payload to behavior to behavior if needed
                else:
                    print(f"[MissionController] Trigger {trigger_key} fired, but another trigger was already processed.")
            # else:
            #     print(f"[MissionController] Ignored trigger '{trigger_key}' (not valid for current phase).")

    async def _wait_for_shutdown_signal(self):
        """Blocks until the mission active event is cleared (shutdown requested)."""
        while self.mission_active_event.is_set():
            await asyncio.sleep(0.1)

    # ====================================================================================
    # NEW: Hub-Driven Control Logic (Simplified Failsafe Version)
    # ====================================================================================

    async def _execute_hub_task(self, payload: Dict[str, Any]):
        """
        Dynamically executes a task commanded by the Hub,
        replacing the current behavior without changing phase.
        """
        # Don't accept commands while transitioning
        if self._is_transitioning:
            print("[MissionController] Ignoring Hub command (transitioning).")
            return

        async with self._transition_lock:
            self._is_transitioning = True # Lock

            try:
                hub_phase_name = payload.get("phase")
                hub_task_data = payload.get("task")

                # Check 1: Is this command still for our current phase?
                current_phase_key = MISSION_ENUM_TO_PHASE[self.mission_state.current]
                if hub_phase_name != current_phase_key:
                    print(f"[MissionController] Ignoring Hub command for old phase '{hub_phase_name}' (current: '{current_phase_key}').")
                    self._is_transitioning = False
                    return

                print(f"[MissionController] Receiving new Hub-driven task for phase '{hub_phase_name}'")

                # 1. Stop current behavior
                if self.current_behavior:
                    await self.current_behavior.stop()

                # 2. Create new Task object from Hub payload
                # This bypasses the local mission.json!
                from ..g1_mission_definition.phase import Task
                hub_task = Task(**hub_task_data)

                # 3. Create and start new behavior
                # FIX: Passing 7 arguments cleanly
                self.current_behavior = self.behavior_factory.create(
                    hub_task,
                    self.mission_state,
                    self.flight_controller,
                    self.mqtt,
                    self.mqtt._client_id,
                    self.config,
                    self.drone_role # <--- ARGUMENT 7
                )
                await self.current_behavior.start()

                # 4. Mark that we are now Hub-driven
                self.is_hub_driven = True
                print(f"[MissionController] Hub-driven behavior '{hub_task.action}' is active.")

            except Exception as e:
                print(f"[MissionController] Error executing Hub task: {e}")
            finally:
                self._is_transitioning = False # Unlock
                # Fire a "no-op" trigger to unblock the main loop
                # This tells the main loop to re-evaluate
                self._fired_trigger = "hub_command:new_task"
                self._trigger_event_fired.set()

    # ====================================================================================
    # Phase-specific listeners (evolved from old wait_for_trigger)
    # ====================================================================================

    def _create_phase_listeners(self, transitions: Dict[str, Dict[str, str]]) -> List[asyncio.Task]:
        """
        Creates and returns a list of asyncio.Tasks for phase-specific
        triggers (on_state, on_timeout).
        """
        tasks = []
        for trigger_key in transitions.keys():
            if trigger_key.startswith("on_state:"):
                state_name = trigger_key.split(":", 1)[1]
                tasks.append(asyncio.create_task(
                    self._monitor_state_changes(trigger_key, state_name),
                    name=f"state_{state_name}"
                ))
            elif trigger_key.startswith("on_timeout:"):
                try:
                    timeout_seconds = float(trigger_key.split(":", 1)[1])
                    tasks.append(asyncio.create_task(
                        self._wait_timeout(trigger_key, timeout_seconds),
                        name=f"timeout_{timeout_seconds}"
                    ))
                except ValueError:
                    print(f"[MissionController] WARNING: Invalid timeout '{trigger_key}'. Ignoring.")
        return tasks

    async def _monitor_state_changes(self, trigger_key: str, state_name: str):
        """
        Monitors MissionState for a specific state change.
        """
        # Create a future that resolves when the state changes
        state_future = asyncio.Future()

        def state_listener(new_state: MissionStateEnum):
            """Called when mission state changes."""
            if new_state.name.upper() == state_name.upper():
                if not state_future.done():
                    state_future.set_result(True)

        # 1. Add listener
        self.mission_state.add_listener(state_listener)
        self._state_listeners.append(state_listener) # For cleanup

        # 2. Check current state
        if self.mission_state.current.name.upper() == state_name.upper():
            if not state_future.done():
                state_future.set_result(True) # Resolve immediately

        try:
            # Wait for the state change
            await state_future
            # Fire the trigger
            await self._check_and_fire_trigger(trigger_key)
        except asyncio.CancelledError:
            pass # Task was cancelled, just exit
        finally:
            # Cleanup listener
            self.mission_state.remove_listener(state_listener)
            if state_listener in self._state_listeners:
                self._state_listeners.remove(state_listener)

    async def _wait_timeout(self, trigger_key: str, timeout_seconds: float):
        """
        Waits for a specified timeout and fires the trigger.
        """
        try:
            await asyncio.sleep(timeout_seconds)
            # Timeout elapsed, fire the trigger
            await self._check_and_fire_trigger(trigger_key)
        except asyncio.CancelledError:
            pass # Task was cancelled, just exit

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
        print(f"[MissionController] Event manually triggered: {event_name}")
        await self._check_and_fire_trigger(trigger_key)
