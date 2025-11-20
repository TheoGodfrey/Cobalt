import asyncio
import argparse
import numpy as np
import yaml
import math
import time
import signal # <-- ADDED for Bug 4
import platform # <-- ADDED for Bug 4

# Core Infrastructure
from core.bus import MessageBus
from core.supervisor import ServiceSupervisor
from core.comms import Comms
from core.types import DroneState, TelemetryMessage

# Logic & State
from core.state.world_state import WorldState
from core.mission.manager import MissionManager
from core.maths.solver import ProbabilisticSolver

# Hardware & Safety
from drone.hardware.hal import MockHAL, MavlinkHAL
from drone.safety.safety_monitor import SafetyMonitor

# Vision (The parts we removed earlier)
from drone.hardware.camera import CameraSensor
from drone.hardware.lidar import LidarSensor
from core.detectors.thermal.manager import DetectorManager
from core.detectors.visual.colour import ColorBlobDetector
from core.maths.georeference import Georeferencer

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--hal", default="mock")
    parser.add_argument("--mission", default="missions/MOB_mission.yaml")
    args = parser.parse_args()

    print(f"[Main] Initializing Drone {args.id}...")

    # 1. Infrastructure (The Nervous System)
    bus = MessageBus()
    supervisor = ServiceSupervisor(bus)
    
    # 2. State & Configuration (The Brain)
    world = WorldState()
    world.my_id = args.id
    
    try:
        with open(args.mission) as f: mission_plan = yaml.safe_load(f)
        manager = MissionManager(mission_plan, world, ProbabilisticSolver(), SafetyMonitor(world))
    except Exception as e:
        print(f"[Main] Error loading mission: {e}")
        manager = None

    # 3. Hardware (The Body)
    if args.hal == "mavlink":
        hal = MavlinkHAL("udp:127.0.0.1:14550")
        camera = CameraSensor(mode="real")
        lidar = LidarSensor(mode="real")
    else:
        hal = MockHAL()
        camera = CameraSensor(mode="mock")
        lidar = LidarSensor(mode="mock")

    # Shared Objects
    solver = ProbabilisticSolver() # Control Math
    safety = SafetyMonitor(world)  # Safety Limits
    comms = Comms(args.id, bus)    # Radio

    # --- Signal Handlers (Required for Bug 4 Fix) ---
    def emergency_kill(signum, frame):
        print(f"\n[Main] Signal {signum} received. Initiating emergency shutdown.")
        if hal: hal.land() # Attempt a controlled landing
        # Stop the asyncio loop for graceful exit
        asyncio.get_running_loop().stop()
        
    def handle_rtl(signum, frame):
        print(f"\n[Main] Signal {signum} received. Returning to Launch (RTL).")
        if hal: hal.return_to_launch()

    def handle_land(signum, frame):
        print(f"\n[Main] Signal {signum} received. Landing Now (LAND).")
        if hal: hal.land()

    # --- Bug 4 Fix: Signal Registration (Replaced lines 68-79) ---
    signal.signal(signal.SIGINT, emergency_kill)
    signal.signal(signal.SIGTERM, emergency_kill)

    if platform.system() != "Windows":
        signal.signal(signal.SIGUSR1, handle_rtl)
        signal.signal(signal.SIGUSR2, handle_land)
        print("[Main] Listening for SIGUSR1 (RTL) and SIGUSR2 (LAND).")
    else:
        print("[Main] On Windows: Use MQTT commands for RTL/LAND (no SIGUSR signals)")
        
    # ---------------------------------------------------------
    # SERVICE 1: Guidance (The Pilot)
    # Priority: HIGH. Must never block.
    # ---------------------------------------------------------
    async def service_guidance():
        print("[Guidance] Service Started.")
        if manager: manager.start()
        await hal.connect()
        
        while True:
            t0 = asyncio.get_running_loop().time()
            
            # A. Get Strict State
            state: DroneState = hal.get_state()

            # --- BUG FIX 5: Manual Override Pause/Resume ---
            if manager:
                # 1. Detect MANUAL override and pause mission state
                if state.mode == "MANUAL" and not manager.paused:
                    manager.pause_mission("manual_override")
                # 2. Detect GUIDED resume after manual override
                elif state.mode == "GUIDED" and manager.paused:
                    manager.resume_mission()
            # -----------------------------------------------
            
            # B. Update Mission Triggers (Did we find it?)
            # Skip mission checks if we are manually flown or paused
            if manager and not manager.paused: 
                manager.check_triggers(state)
            
            # C. Solve for Velocity
            if manager and not manager.paused:
                # Mission is active: calculate command from solver
                raw_cmd = solver.solve(state)
            else:
                # Mission is paused (MANUAL, or awaiting resume): command zero velocity
                raw_cmd = np.zeros(3)
                
            # D. Safety Filter (Dynamic Home & Geofence)
            safe_cmd = safety.filter_velocity(raw_cmd, state)
            
            # E. Actuate
            hal.send_velocity_command(safe_cmd)
            
            # F. Telemetry (Fire & Forget)
            telem = TelemetryMessage(
                id=args.id,
                pos=state.position_local.tolist(),
                vel=state.velocity.tolist(),
                batt=state.battery,
                hdg=state.heading,
                mode=state.mode,
                phase=manager.current_phase_idx if manager else 0
            )
            # Re-added 'posture' from the previously fixed version (Bug 3 context)
            telem_dict = telem.model_dump()
            telem_dict['posture'] = manager.current_posture if manager else "none"
            
            await bus.publish("comms/send", {"topic": "fleet/telemetry", "payload": telem_dict})
            
            # G. Heartbeat
            await bus.publish("system/heartbeat", {"service": "guidance"})
            
            # H. Loop Timing (Strict 10Hz)
            elapsed = asyncio.get_running_loop().time() - t0
            await asyncio.sleep(max(0, 0.1 - elapsed))

    # ---------------------------------------------------------
    # SERVICE 2: Vision (The Co-Pilot/Observer)
    # Priority: MEDIUM. Can run slower (e.g., 5Hz).
    # ---------------------------------------------------------
    async def service_vision():
        print("[Vision] Service Started.")
        
        # Initialize Detectors locally (Thread-safe)
        detector_mgr = DetectorManager()
        detector_mgr.add_detector(ColorBlobDetector(color_range="orange"))
        georef = Georeferencer(fov_deg_x=60.0)
        
        while True:
            t0 = time.time()
            
            # A. Get Data (Blocking I/O is okay here, won't stop guidance)
            # Note: In real async, we'd use run_in_executor for heavy CV calls
            frame = camera.get_frame()
            state = hal.get_state() # Get latest state for georeferencing
            
            # B. Process
            detections = detector_mgr.process(frame)
            
            # C. Georeference & Publish
            for d in detections:
                # Simple Pinhole Model
                cx, cy = d.bbox[0] + d.bbox[2]/2, d.bbox[1] + d.bbox[3]/2
                
                # Get Lidar Range (Simulated raycast)
                # ( Simplified: Center ray for now )
                lidar_dist = lidar.get_range_at_angle(0, 0) 
                
                world_pos = georef.pixel_to_world(cx, cy, state, lidar_range=lidar_dist)
                
                if world_pos is not None:
                    # Notify the Bus!
                    print(f"[{args.id}] SEE: {d.label} at {world_pos}")
                    
                    target_msg = {
                        'pos': world_pos.tolist(),
                        'label': d.label,
                        'conf': d.confidence,
                        'id': args.id
                    }
                    
                    # Publish locally (updates WorldState)
                    # And remote (Comms will pick this up via listener if we wired it)
                    await bus.publish("vision/detection", target_msg)
                    await bus.publish("comms/send", {"topic": "fleet/target", "payload": target_msg})

            # D. Heartbeat
            await bus.publish("system/heartbeat", {"service": "vision"})
            
            # Run at 5Hz (slower than guidance)
            await asyncio.sleep(0.2)

    # ---------------------------------------------------------
    # SERVICE 3: Comms (The Radio Operator)
    # ---------------------------------------------------------
    async def service_comms():
        await comms.run()

    # ---------------------------------------------------------
    # SERVICE 4: Listener (The Ears)
    # Handles incoming data from the Bus
    # ---------------------------------------------------------
    async def service_listener():
        # Updates Home Position if Hub moves
        async def on_home_update(data):
            safety.update_home(data['pos'])
        
        # Updates World State with detections from OTHER drones
        async def on_network_target(data):
            world.targets.update(data['pos'], data['label'], data['conf'])

        # --- Handler for fleet position synchronization (Bug 3 Fix) ---
        async def on_fleet_telemetry(data):
            drone_id = data.get('id')
            pos = data.get('pos') # List of floats
            
            # Skip my own telemetry
            if drone_id and pos and (drone_id != args.id):
                # We assume WorldState has 'fleet_positions' initialized
                world.fleet_positions[drone_id] = np.array(pos)
                print(f"[Fleet] Updated {drone_id} position: {pos}")
        # ----------------------------------------------------------
            
        bus.subscribe("fleet/home", on_home_update)
        bus.subscribe("fleet/target", on_network_target)
        # --- Subscribe to telemetry for swarm equation (Bug 3 Fix) ---
        bus.subscribe("fleet/telemetry", on_fleet_telemetry)
        # ----------------------------------------------------------
        
        while True:
            await bus.publish("system/heartbeat", {"service": "listener"})
            await asyncio.sleep(1.0)

    # 4. Register & Launch
    # This is where we prevent the "God Script" problem. 
    # If Vision crashes, Guidance lives.
    supervisor.register("comms", service_comms)
    supervisor.register("guidance", service_guidance)
    supervisor.register("vision", service_vision)
    supervisor.register("listener", service_listener)

    print("[Main] All Services Registered. Starting Supervisor.")
    await supervisor.start()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")