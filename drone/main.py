import argparse
import yaml
import time
import asyncio
import numpy as np
import math
import signal
import sys
from pathlib import Path

# Import Core
from core.state.world_state import WorldState
from core.maths.solver import ProbabilisticSolver
from core.mission.manager import MissionManager
from core.comms import Comms
from drone.safety.safety_monitor import SafetyMonitor
from drone.hardware.hal import MockHAL, MavlinkHAL

# --- NEW IMPORTS FOR VISION/LIDAR ---
from drone.hardware.camera import CameraSensor
from drone.hardware.lidar import LidarSensor
from core.detectors.thermal.manager import DetectorManager
from core.detectors.visual.colour import ColorBlobDetector
from core.maths.georeference import Georeferencer
# ------------------------------------

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--mission", default="missions/MOB_mission.yaml")
    parser.add_argument("--hal", default="mock")
    args = parser.parse_args()

    # Force unbuffered output for robust logging
    print(f"[Main] Starting COBALT Drone: {args.id}", flush=True)

    base_path = Path(__file__).parent.parent
    
    # --- LOAD CONFIG ---
    try:
        with open(base_path / "config/fleet_config.yaml") as f: fleet_conf = yaml.safe_load(f)
        my_conf = fleet_conf.get('fleet', {}).get(args.id, {'connection': 'udp:127.0.0.1:14550'})
        my_role = my_conf.get('type', 'scout') 
    except FileNotFoundError:
        fleet_conf = {'world': {}}
        my_conf = {'connection': 'udp:127.0.0.1:14550'}
        my_role = 'scout'

    try:
        with open(args.mission) as f: mission_plan = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"[Main] CRITICAL: Mission file {args.mission} not found.", flush=True)
        return

    # --- INITIALIZE STATE & PHYSICS ---
    world = WorldState(fleet_conf.get('world'))
    world.my_id = args.id 
    world.fleet_positions = {} 

    solver = ProbabilisticSolver(config={'max_speed_xy': 20.0})
    safety = SafetyMonitor(world, config={'min_altitude': 5.0})
    
    # --- HARDWARE ABSTRACTION LAYER & SENSORS ---
    if args.hal == "mavlink":
        hal = MavlinkHAL(my_conf['connection'])
        camera = CameraSensor(mode="real")
        lidar = LidarSensor(mode="real")
    else:
        hal = MockHAL()
        camera = CameraSensor(mode="mock")
        lidar = LidarSensor(mode="mock")

    # --- SETUP DETECTION PIPELINE ---
    detector_manager = DetectorManager()
    # Add the new Visual Detector for Life Jackets (International Orange)
    detector_manager.add_detector(ColorBlobDetector(color_range="orange"))
    
    # Initialize Georeferencer (Sensor Fusion Engine)
    georef = Georeferencer(fov_deg_x=60.0)
    # --------------------------------

    # --- SIGNAL HANDLERS ---
    def emergency_shutdown(signum, frame):
        print(f"\n[{args.id}] CAUGHT SIGNAL {signum}. INITIATING EMERGENCY LANDING.", flush=True)
        hal.land()
        sys.exit(0)

    signal.signal(signal.SIGINT, emergency_shutdown)
    signal.signal(signal.SIGTERM, emergency_shutdown)

    # --- COMMUNICATIONS ---
    comms = Comms(args.id)

    manager = MissionManager(
        plan=mission_plan,
        world=world,
        solver=solver,
        safety=safety,
        my_role=my_role
    )

    await hal.connect()
    comms.connect()

    loop = asyncio.get_running_loop()

    # Comms Callbacks
    def safe_update_telemetry(msg):
        if msg['id'] != args.id:
            world.fleet_positions[msg['id']] = np.array(msg['pos'])

    def safe_update_target(msg):
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
    
    def safe_mission_command(msg):
        cmd = msg.get('command')
        if cmd == 'NEXT_PHASE':
            manager.trigger_manual_override()
        elif cmd == 'RTL':
            print(f"[{args.id}] Received Global RTL Command.", flush=True)
            hal.return_to_launch()
        elif cmd == 'LAND':
            print(f"[{args.id}] Received Global LAND Command.", flush=True)
            hal.land()

    comms.sub("fleet/telemetry", lambda m: loop.call_soon_threadsafe(safe_update_telemetry, m))
    comms.sub("fleet/target", lambda m: loop.call_soon_threadsafe(safe_update_target, m))
    comms.sub(f"fleet/{args.id}/command", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))
    comms.sub("fleet/broadcast", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))

    print("[Main] Auto-arming and Starting Mission...", flush=True)
    manager.start() 

    # --- MAIN CONTROL LOOP ---
    dt = 0.1
    last_print_time = 0.0 # For robust logging

    while True:
        t0 = time.time()
        state = hal.get_state()
        
        # --- 1. SENSOR FUSION LOOP (Visual + Lidar) ---
        # Grab Frame
        frame = camera.get_frame()
        
        # Run Detectors
        detections = detector_manager.process(frame)
        
        for d in detections:
            # We found something visually. Now we need to know WHERE it is.
            cx, cy = d.bbox[0] + d.bbox[2]/2, d.bbox[1] + d.bbox[3]/2
            
            # Calculate angular offset for accurate LIDAR query
            ang_x = math.atan2(cx - 320, georef.f_px)
            ang_y = math.atan2(cy - 240, georef.f_px)
            
            # Query Lidar for range at this specific angle
            lidar_dist = lidar.get_range_at_angle(ang_x, ang_y)
            
            # Fuse Data (Visual + Lidar + GPS + Attitude)
            world_pos = georef.pixel_to_world(
                cx, cy, 
                state, 
                lidar_range=lidar_dist
            )
            
            if world_pos is not None:
                # Sanity Check: Is the target near sea level? (z=0 approx)
                # Allow +/- 5m for waves/tide/sensor noise
                if abs(world_pos[2]) < 5.0:
                    print(f"[{args.id}] *** CONTACT *** {d.label} at {world_pos} (Conf: {d.confidence:.2f})", flush=True)
                    
                    # Broadcast to Fleet
                    target_msg = {
                        'pos': world_pos.tolist(),
                        'label': d.label,
                        'conf': d.confidence,
                        'lidar_range': lidar_dist,
                        'id': args.id
                    }
                    comms.pub("fleet/target", target_msg)
                    
                    # Update local world state
                    world.targets.update(world_pos, d.label, d.confidence)
        # ----------------------------------------------

        # Update Probability Map (Sensor Coverage)
        altitude = -state.position_local[2]
        if altitude > 1.0: 
            world.probability.update_from_sensor(
                x=state.position_local[0], y=state.position_local[1],
                z_drone=altitude, fov_angle_deg=60.0
            )
        
        # Evolve World State (Wind Drift)
        wind_est = world.wind.get_at(state.position_local)
        world.probability.evolve(dt, drift_vector_ms=wind_est)

        # Check Phase Triggers (e.g., did we find the target?)
        manager.check_triggers(state)

        # Solve for Next Velocity Command
        raw_cmd = solver.solve(state)
        solver_debug = solver.get_debug_info() 
        
        # Safety Filter
        safe_cmd = safety.filter_velocity(raw_cmd, state)
        hal.send_velocity_command(safe_cmd)
        
        # Publish Telemetry
        telem_packet = {
            'id': args.id, 't': time.time(),
            'pos': state.position_local.tolist(), 'vel': state.velocity.tolist(),
            'hdg': math.degrees(state.heading), 'batt': state.battery,
            'mode': state.mode, 'phase': manager.current_phase_idx,
            'posture': manager.current_posture, 'wind': wind_est.tolist(),
        }

        if solver_debug:
            telem_packet['debug'] = {
                'grad_travel': solver_debug.gradient_travel.tolist(),
                'grad_search': solver_debug.gradient_search.tolist(),
                'grad_wind': solver_debug.gradient_wind.tolist(),
                'cost': solver_debug.cost_value
            }

        comms.pub("fleet/telemetry", telem_packet)

        # --- ROBUST LOGGING ---
        # Replaced flaky modulo check with explicit timer
        if time.time() - last_print_time > 2.0:
            last_print_time = time.time()
            print(f"[{args.id}] Phase:{manager.current_phase_idx} | Batt:{state.battery:.0f}% | Mode:{state.mode}", flush=True)

        # Maintain Loop Rate
        elapsed = time.time() - t0
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass