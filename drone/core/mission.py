"""
Asynchronous, event-driven, P2P mission controller.
(Refactored for COBALT P2P Architecture)

This file is based on the original `mission.py` from the `drone-mob` repository
[cite: theogodfrey/drone-mob/Drone-MOB-ca18888baa3e251f77acf01f881b5934e4ab4e7f/v_0.2/scout_drone/core/mission.py]
and has been significantly modified to implement the decentralized P2P architecture
described in the "Cobalt drone" document [cite: Cobalt drone] and the
distributed AI search from the patent [cite: patent_1_probabilistic_search_delivery_improved.md].
"""

import asyncio
import traceback
from .drone import Drone
from .position import Position
from .logger import MissionLogger
from .state_machine import MissionStateMachine, MissionPhase
from .config_models import Settings 
from .behaviors import SearchBehavior, DeliveryBehavior
from .cameras.dual_camera import DualCameraSystem 
from .telemetry_logger import TelemetryLogger
from .comms import MqttClient

# --- NEW: Import AI logic. Moved from coordinator to drone core ---
# This assumes the file `coordinator/prob_search.py` is moved to `v_0.2/scout_drone/core/ai/prob_search.py`
try:
    from .ai.prob_search import ProbabilisticSearchManager
except ImportError:
    print("WARNING: Could not import ProbabilisticSearchManager. Scout AI search will not be available.")
    print("Ensure 'prob_search.py' is moved to 'core/ai/prob_search.py'")
    ProbabilisticSearchManager = None

class MissionController:
    """
    Asynchronous mission controller for a *single* drone.
    This class *is* the model for the state machine.
    It now listens for fleet-wide P2P events and decides how to act
    based on its configured role, as described in [cite: Cobalt drone].
    
    It maintains its own local copy of the probabilistic map and updates
    it based on its own actions and "gossip" messages from the fleet
    [cite: patent_1_probabilistic_search_delivery_improved.md].
    """
    
    def __init__(self, 
                 drone: Drone, 
                 dual_camera: DualCameraSystem | None,
                 search_strategies: dict,
                 flight_strategies: dict,
                 config: Settings,
                 logger: MissionLogger,
                 mqtt_client: MqttClient):
        
        self.drone = drone
        self.dual_camera = dual_camera
        self.search_strategies = search_strategies
        self.flight_strategies = flight_strategies
        self.config = config
        self.logger = logger
        self.mqtt = mqtt_client
        self.target = None
        self.target_position = None
        self.current_mission_type = "IDLE"
        self.role = self._get_role() # <-- NEW: Store role
        self.high_battery_threshold = 80.0 # From Cobalt doc
        
        self.telemetry_logger = None 
        self.search_behavior = None
        self.delivery_behavior = None
        self.prob_search_manager = None # <-- NEW: AI/Prob search manager

        # Only create camera-dependent components if cameras exist
        if self.dual_camera:
            self.telemetry_logger = TelemetryLogger(
                log_dir=f"{config.logging.log_dir}/{drone.id}"
            )
            self.search_behavior = SearchBehavior(
                drone=drone,
                dual_camera=self.dual_camera,
                search_strategy=None, # Will be set by mission type
                flight_strategy=flight_strategies['direct'],
                config=config
            )
        
        self.delivery_behavior = DeliveryBehavior(
            drone=drone,
            flight_strategy=flight_strategies['precision_hover'],
            config=config.precision_hover
        )
        
        # --- REFACTORED: All drones now load the AI module ---
        # All drones participate in the shared probabilistic model,
        # even if they just listen (like Payload).
        if ProbabilisticSearchManager:
            self.logger.log(f"Role {self.role.upper()}: Initializing local ProbabilisticSearchManager.")
            self.prob_search_manager = ProbabilisticSearchManager(
                config.prob_search,
                config.strategies.search.area
            )
        else:
            self.logger.log(f"Role {self.role.upper()}: AI module not loaded. Probabilistic search disabled.", "error")
        # ----------------------------------------
        
        self.state_machine = MissionStateMachine(self, mqtt_client)
        
        self.logger.log(f"Initialized mission for {drone.id}", "info")
        self.logger.log(f"Hardware Role: {self.role.upper()}", "info")

    def _get_role(self) -> str:
        """Get the drone's innate hardware role from the config."""
        for d in self.config.drones:
            if d.id == self.drone.id:
                return d.role
        self.logger.log(f"FATAL: Drone ID '{self.drone.id}' not found in config.drones", "error")
        return "unknown"

    async def run(self) -> None:
        """
        Execute the main asynchronous P2P mission loop.
        Listens for global events and runs the health monitor.
        """
        try:
            # --- REFACTORED: Subscribe to global P2P topics ---
            await self.mqtt.subscribe("mission/start")
            await self.mqtt.subscribe("fleet/event/confirmation")
            await self.mqtt.subscribe("fleet/event/target_found")
            # --- NEW: P2P Map Sharing Topic ---
            await self.mqtt.subscribe("fleet/map/update")
            
            self.logger.log(f"Listening for P2P events...", "info")
            # --------------------------------------------------
            
            await self.mqtt.publish("fleet/connect", {"drone_id": self.drone.id, "role": self.role})

            await asyncio.gather(
                self._p2p_event_listener(), # Listens for fleet messages
                self._health_monitor()      # Monitors self and publishes telemetry
            )

        except (KeyboardInterrupt, asyncio.CancelledError):
            self.logger.log("Mission interrupted by user/system", "warning")
            await self.trigger_emergency(event=None)
        except Exception as e:
            self.logger.log(f"Fatal mission error: {e}", "error")
            traceback.print_exc()
            await self.trigger_emergency(event=None)
        finally:
            self.logger.log("Cleaning up resources...", "info")
            if self.telemetry_logger:
                self.telemetry_logger.close()
            if self.drone.telemetry.is_connected: await self.drone.disconnect()
            if self.dual_camera and self.dual_camera.connected: await self.dual_camera.disconnect()
            self.logger.log("Cleanup complete.", "info")
    
    async def _p2p_event_listener(self):
        """
        Main P2P loop that waits for global events and triggers
        autonomous role-based actions based on [cite: Cobalt drone].
        """
        async for topic, payload in self.mqtt.listen():
            try:
                # --- Pre-emption Logic ---
                # High-priority events (e.g., MOB) can interrupt
                # low-priority states (e.g., IDLE, ROLE_UTILITY_TASK).
                
                if topic == "mission/start":
                    mission_type = payload.get("type")
                    self.logger.log(f"Received global mission/start event: {mission_type}", "info")
                    
                    # Store target position if provided (for General Emergency)
                    if "position" in payload and payload["position"] is not None:
                        self.target_position = Position(**payload["position"])

                    # --- Use Case 1: MOB (MAX PRIORITY) ---
                    if mission_type == "MOB_EMERGENCY":
                        # All drones check their role and act
                        if self.role == "scout":
                            self.logger.log("MOB Event: Assuming ROLE_SEARCH_PRIMARY", "info")
                            self.current_mission_type = "MOB_SEARCH"
                            await self.start_mission()
                        
                        elif self.role == "payload":
                            # --- CORRECTED LOGIC ---
                            # Payload drone joins the co-op search, as per the patent.
                            # It is forbidden from UTILITY tasks, not emergency SEARCH_ASSIST.
                            self.logger.log("MOB Event: Assuming ROLE_SEARCH_ASSIST (Payload)", "info")
                            self.current_mission_type = "MOB_SEARCH"
                            await self.start_mission()
                            # --- END CORRECTION ---
                        
                        elif self.role == "utility":
                            self.logger.log("MOB Event: Assuming ROLE_SEARCH_ASSIST", "info")
                            self.current_mission_type = "MOB_SEARCH"
                            # self.search_behavior.search_strategy = self.search_strategies['lawnmower'] # No longer needed, uses AI
                            await self.start_mission()

                    # --- Use Case 2: General Emergencies (L3) ---
                    elif mission_type == "GENERAL_EMERGENCY":
                        if self.role == "scout":
                            self.logger.log("Gen-Emerg Event: Assuming ROLE_EMERGENCY_EYES", "info")
                            self.current_mission_type = "OVERWATCH"
                            await self.start_overwatch_mission()
                        
                        elif self.role == "payload":
                            self.logger.log("Gen-Emerg Event: Assuming ROLE_EMERGENCY_STANDBY", "info")
                            self.current_mission_type = "STANDBY"
                            self.target_position.z = 30.0 # Standby near event
                            await self.start_standby_mission()
                        
                        elif self.role == "utility":
                            self.logger.log("Gen-Emerg Event: Assuming ROLE_EMERGENCY_ASSIST", "info")
                            if self.drone.telemetry.battery > self.config.health.min_battery_patrol_rtl:
                                self.current_mission_type = "OVERWATCH"
                                await self.start_overwatch_mission()
                            else:
                                self.logger.log("Utility battery low, ignoring Gen-Emerg.", "warning")

                    # --- Use Case 3: Utility & Compliance (L1/L2) ---
                    elif mission_type == "UTILITY_HULL_INSPECTION":
                        if self.role == "utility":
                            self.logger.log("Utility Event: Assuming ROLE_UTILITY_TASK", "info")
                            self.current_mission_type = "PATROL"
                            self.search_behavior.search_strategy = self.search_strategies['lawnmower']
                            await self.start_patrol_mission()
                        
                        elif self.role == "scout":
                            # Logic from [cite: Cobalt drone]: "only allows it to accept... if its battery is above a high threshold"
                            if self.drone.telemetry.battery > self.high_battery_threshold:
                                self.logger.log(f"Scout accepting Utility task (battery {self.drone.telemetry.battery}% > {self.high_battery_threshold}%)", "info")
                                self.current_mission_type = "PATROL"
                                self.search_behavior.search_strategy = self.search_strategies['lawnmower']
                                await self.start_patrol_mission()
                            else:
                                self.logger.log(f"Scout battery {self.drone.telemetry.battery}% < {self.high_battery_threshold}%. Ignoring Utility task.", "warning")
                        
                        elif self.role == "payload":
                            # Logic from [cite: Cobalt drone]: "This drone is forbidden from performing COMPLIANCE (Utility) tasks"
                            self.logger.log("Payload role is FORBIDDEN from Utility tasks. Ignoring.", "warning")
                            # Do nothing
                
                # --- P2P Event: Target Handoff ---
                elif topic == "fleet/event/target_found":
                    if self.role == "payload" and self.state in [MissionPhase.ROLE_EMERGENCY_STANDBY, MissionPhase.IDLE]:
                        self.logger.log("Target found by another drone. Assuming ROLE_DELIVERING", "info")
                        self.target_position = Position(**payload["position"])
                        self.current_mission_type = "PAYLOAD_DELIVERY"
                        await self.start_delivery_mission()

                # --- P2P Event: Operator Confirmation ---
                elif topic == "fleet/event/confirmation":
                    target_drone = payload.get("drone_id")
                    # Is this confirmation for *me*?
                    if target_drone == self.drone.id:
                        if payload.get("type") == "OPERATOR_CONFIRM_TARGET":
                            if self.state == MissionPhase.TARGET_PENDING_CONFIRMATION:
                                await self.confirm_target()
                        elif payload.get("type") == "OPERATOR_REJECT_TARGET":
                            if self.state == MissionPhase.TARGET_PENDING_CONFIRMATION:
                                await self.reject_target()
                
                # --- P2P Event: Shared Map Update ---
                elif topic == "fleet/map/update":
                    source_drone = payload.get("drone_id")
                    # Update our map if it's from *another* drone and we have an AI manager
                    if source_drone != self.drone.id and self.prob_search_manager:
                        self.logger.log(f"Received map update from {source_drone}", "debug")
                        self.prob_search_manager.update_map(
                            drone_pos=Position(**payload["position"]),
                            drone_altitude=payload["altitude"],
                            has_detection=payload["has_detection"]
                        )

            except Exception as e:
                self.logger.log(f"Error in P2P listener: {e}", "error")
                traceback.print_exc()


    async def _health_monitor(self):
        """Periodic loop to update telemetry and check health."""
        while True:
            try:
                # Always update telemetry, even on ground, to check battery
                await self.drone.update_telemetry()

                if self.state not in [MissionPhase.IDLE, MissionPhase.PREFLIGHT]:
                    
                    # --- Local Operator Takeover Logic (Level 2) ---
                    is_manual = self.drone.telemetry.state == "MANUAL"
                    is_in_local_control = self.state == MissionPhase.LOCAL_OPERATOR_CONTROL
                    
                    if is_manual and not is_in_local_control:
                        self.logger.log("!!! Local Operator Takeover Detected (Level 2) !!!", "warning")
                        await self.local_operator_takeover()
                    elif not is_manual and is_in_local_control:
                        self.logger.log("Local Operator has released control.", "info")
                        await self.local_operator_release()
                    
                    if not is_in_local_control and not self.drone.is_healthy():
                        self.logger.log("Drone unhealthy, transitioning to EMERGENCY.", "error")
                        await self.trigger_emergency(event=None)
                        continue
                        
                    # Log to file if we have a logger
                    if self.telemetry_logger:
                        await self.telemetry_logger.log_snapshot(
                            mission_state=self.state.value,
                            drone=self.drone,
                            detections=self.search_behavior.get_last_detections() if self.search_behavior else []
                        )
                
                # --- Always publish telemetry for GCS/Hub ---
                telemetry_payload = self.drone.telemetry.model_dump()
                telemetry_payload["mission_phase"] = self.state.value # Add this
                
                await self.mqtt.publish(
                    f"fleet/telemetry/{self.drone.id}",
                    telemetry_payload
                )
                
                await asyncio.sleep(1.0)
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.log(f"Error in health monitor: {e}", "error")
                await self.trigger_emergency(event=None)

    # --- State Machine Callbacks (Updated for P2P/AI) ---

    async def _run_preflight(self, event):
        self.logger.log(f"Entering PREFLIGHT state for {self.current_mission_type}", "info")
        try:
            if not await self.drone.connect():
                 raise Exception("Drone failed to connect.")
            
            # Update telemetry right after connect to get battery
            await self.drone.update_telemetry()
            
            if self.role in ["scout", "utility"]:
                if not self.dual_camera or not await self.dual_camera.connect():
                    raise Exception("Camera system failed to connect.")

            if self.drone.telemetry.battery < self.config.health.min_battery_preflight:
                raise Exception(f"{self.drone.id} - Low battery ({self.drone.telemetry.battery}%)")
            
            self.logger.log("Preflight check complete", "info")
            await self.preflight_success()
        except Exception as e:
            self.logger.log(f"Preflight failed: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_takeoff(self, event):
        self.logger.log(f"Entering TAKEOFF state", "info")
        try:
            # Determine takeoff altitude based on mission
            if self.current_mission_type == "PATROL":
                alt = self.config.lawnmower.patrol_altitude
            elif self.current_mission_type == "OVERWATCH":
                alt = self.target_position.z + self.config.orbit.altitude_offset
            elif self.current_mission_type == "STANDBY":
                alt = self.target_position.z
            elif self.current_mission_type == "PAYLOAD_DELIVERY":
                alt = 30.0
            else: # MOB_SEARCH
                # --- NEW: Use AI config altitude ---
                # Both Scout and Utility use the AI search altitude
                alt = self.config.prob_search.search_altitude
            
            if not await self.drone.takeoff(alt):
                raise Exception("Takeoff command failed.")
            self.logger.log(f"Takeoff complete to {alt}m", "info")
            await self.takeoff_success()
        except Exception as e:
            self.logger.log(f"Takeoff failed: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_search_step(self, event):
        self.logger.log(f"Entering SEARCHING state (Role: {self.state.value})", "info")
        if not self.search_behavior or not self.prob_search_manager:
            self.logger.log("Error: Cannot enter SEARCHING (no camera/behavior or AI module).", "error")
            await self.trigger_emergency(event=event)
            return

        while self.state in [MissionPhase.ROLE_SEARCH_PRIMARY, MissionPhase.ROLE_SEARCH_ASSIST]:
            try:
                detection = None
                
                # --- REFACTORED: AI-Driven Search for BOTH Scout and Utility ---
                # This is the true co-op search. Both roles use the AI.
                
                # 1. Evolve map for drift
                self.prob_search_manager.evolve_map(dt=1.0) # Assume 1s loop
                
                # 2. Get next AI waypoint
                next_wp = self.prob_search_manager.get_next_search_waypoint()
                self.logger.log(f"[AI Search] Role {self.role.upper()} flying to new waypoint: {next_wp}", "debug")
                await self.drone.go_to(next_wp)
                
                # 3. Scan at waypoint
                # We assume search_step() is a point-scan or short hover
                should_continue, detection = await self.search_behavior.search_step()
                
                # 4. Update *local* AI map
                self.prob_search_manager.update_map(
                    drone_pos=self.drone.telemetry.position,
                    drone_altitude=self.drone.telemetry.position.z,
                    has_detection=bool(detection)
                )
                
                # 5. --- P2P SHARE ---
                # Broadcast our finding to the fleet
                self.logger.log(f"Broadcasting map update to fleet/map/update", "debug")
                await self.mqtt.publish("fleet/map/update", {
                    "drone_id": self.drone.id,
                    "position": self.drone.telemetry.position.model_dump(),
                    "altitude": self.drone.telemetry.position.z,
                    "has_detection": bool(detection)
                })

                # --- Common logic ---
                if detection:
                    self.target = detection
                    await self.target_sighted()
                    break
                elif not should_continue:
                    await self.search_complete_negative()
                    break
                
                await asyncio.sleep(0.5)
            
            except Exception as e:
                self.logger.log(f"Error during search step: {e}", "error")
                await self.trigger_emergency(event=event)
                break
    
    async def _run_patrol(self, event):
        self.logger.log(f"Entering PATROL state (Role: {self.state.value})", "info")
        if not self.search_behavior:
            await self.trigger_emergency(event=event)
            return
            
        self.search_behavior.search_strategy = self.search_strategies['lawnmower']
        
        while self.state == MissionPhase.ROLE_UTILITY_TASK:
            try:
                if self.drone.telemetry.battery < self.config.health.min_battery_patrol_rtl:
                    self.logger.log("Patrol battery low, returning to home.", "warning")
                    await self.patrol_battery_low()
                    break
                
                should_continue, detection = await self.search_behavior.search_step()
                
                if detection:
                    self.logger.log("Sighting during patrol, logging but continuing.", "info")
                    # In a real system, might publish this as a low-priority event
                
                if not should_continue:
                    await self.patrol_complete()
                    break
                    
                await asyncio.sleep(0.5)
            except Exception as e:
                self.logger.log(f"Error during patrol step: {e}", "error")
                await self.trigger_emergency(event=event)
                break

    async def _run_overwatch(self, event):
        self.logger.log(f"Entering OVERWATCH state (Role: {self.state.value})", "info")
        try:
            self.search_behavior.search_strategy = self.search_strategies['orbit']
            self.search_behavior.search_strategy.set_center(self.target_position)

            while self.state == MissionPhase.ROLE_EMERGENCY_EYES:
                should_continue, detection = await self.search_behavior.search_step()
                if detection:
                    self.logger.log("Sighting during overwatch, logging.", "info")
                
                # Overwatch continues until a new event (e.g., MOB) pre-empts it
                # or the operator sends a "return" command (not implemented)
                # or it's pre-empted by low battery.
                await asyncio.sleep(1.0)
                
        except Exception as e:
            self.logger.log(f"Error during overwatch: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_standby(self, event):
        self.logger.log(f"Entering STANDBY state (Role: {self.state.value})", "info")
        try:
            await self.drone.go_to(self.target_position)
            self.logger.log(f"Holding position at {self.target_position}", "info")
            # Drone will just hover here. The `_p2p_event_listener`
            # is waiting for the `fleet/event/target_found` message
            # and `fleet/map/update` to keep its local map "hot".
            while self.state == MissionPhase.ROLE_EMERGENCY_STANDBY:
                await asyncio.sleep(1.0)
                
        except Exception as e:
            self.logger.log(f"Error during standby: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_local_operator_takeover(self, event):
        self.logger.log(f"State: {self.state.value}. Handing off control.", "warning")
        # Stop any autonomous behavior
        await self.drone.hold()
        if self.telemetry_logger:
            self.telemetry_logger.pause()

    async def _run_local_operator_release(self, event):
        self.logger.log(f"State: {self.state.value}. Resuming autonomous RTL.", "info")
        if self.telemetry_logger:
            self.telemetry_logger.resume()
        # Default safety behavior is to return home
        await self._run_return_to_home(event)


    async def _request_operator_confirmation(self, event):
        self.logger.log("Target sighted. Requesting operator confirmation...", "info")
        # Publish P2P event for GCS/Hub to see
        await self.mqtt.publish(f"fleet/event/{self.drone.id}", {
            "type": "PENDING_CONFIRMATION",
            "data": { 
                "drone_id": self.drone.id, 
                "position": self.target.position_world.model_dump(), 
                "confidence": self.target.confidence 
            }
        })
        
    async def _handle_rejection(self, event):
        self.logger.log("Operator rejected target. Resuming search.", "warning")
        self.target = None
        # State machine automatically transitions back to searching
        
    async def _request_delivery(self, event):
        """Scout confirms target and requests delivery."""
        self.logger.log("Target confirmed. Broadcasting TARGET_FOUND event.", "info")
        
        # --- REFACTORED: Publish global TARGET_FOUND event ---
        # The PAYLOAD drone will be listening for this.
        await self.mqtt.publish(f"fleet/event/target_found", {
            "position": self.target.position_world.model_dump(),
            "source_drone": self.drone.id
        })
        # ------------------------------------------------
        
        # Scout's job is done, it goes home.
        await self.delivery_request_sent() 

    async def _run_payload_delivery(self, event):
        self.logger.log(f"Entering DELIVERY state (Role: {self.state.value})", "info")
        try:
            await self.delivery_behavior.run(self.target_position)
            self.logger.log("Delivery complete.", "info")
            await self.delivery_complete()
        except Exception as e:
            self.logger.log(f"Error during delivery: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_return_to_home(self, event):
        self.logger.log(f"Entering RETURNING state", "info")
        try:
            if not await self.drone.return_to_home():
                raise Exception("RTL command failed.")
            self.logger.log("RTL command sent. Monitoring arrival.", "info")
            
            while self.state == MissionPhase.RETURNING:
                if self.drone.telemetry.is_home:
                    self.logger.log("Arrived home.", "info")
                    await self.arrived_home()
                    break
                await asyncio.sleep(1.0)
                
        except Exception as e:
            self.logger.log(f"RTL failed: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_land(self, event):
        self.logger.log(f"Entering LANDING state", "info")
        try:
            if not await self.drone.land():
                raise Exception("Land command failed.")
            self.logger.log("Landed.", "info")
            await self.land_complete()
        except Exception as e:
            self.logger.log(f"Land failed: {e}", "error")
            await self.trigger_emergency(event=event)

    async def _run_emergency_land(self, event):
        self.logger.log(f"Entering EMERGENCY state", "critical")
        await self.drone.land() # Force immediate landing
        self.logger.log("Emergency land complete.", "critical")
        await self.reset_from_emergency()

    async def _log_mission_summary(self, event):
        self.logger.log("Entering COMPLETED state", "info")
        self.logger.log("Mission summary: [Summary details...]", "info")
        if self.target:
            self.logger.log(f"Target found at {self.target.position_world}", "info")
        else:
            self.logger.log("No target was confirmed.", "info")
        await self.mission_finished()

