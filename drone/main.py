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
from drone.hardware.camera import CameraSensor
from drone.hardware.lidar import LidarSensor
from core.detectors.thermal.manager import DetectorManager
from core.detectors.visual.colour import ColorBlobDetector
from core.maths.georeference import Georeferencer

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--mission", default="missions/MOB_mission.yaml")
    parser.add_argument("--hal", default="mock")
    args = parser.parse_args()

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

    # --- INITIALIZE ---
    world = WorldState(fleet_conf.get('world'))
    world.my_id = args.id 
    world.fleet_positions = {} 

    solver = ProbabilisticSolver(config={'max_speed_xy': 20.0})
    safety = SafetyMonitor(world, config={'min_altitude': 5.0})
    
    if args.hal == "mavlink":
        hal = MavlinkHAL(my_conf['connection'])
        camera = CameraSensor(mode="real")
        lidar = LidarSensor(mode="real")
    else:
        hal = MockHAL()
        camera = CameraSensor(mode="mock")
        lidar = LidarSensor(mode="mock")

    detector_manager = DetectorManager()
    detector_manager.add_detector(ColorBlobDetector(color_range="orange"))
    georef = Georeferencer(fov_deg_x=60.0)
    
    # --- INTERRUPT HANDLERS ---
    
    def handle_rtl(signum, frame):
        """Interrupt: Return to Base"""
        print(f"\n[{args.id}] INTERRUPT: RETURNING TO BASE (RTL).", flush=True)
        hal.return_to_launch()

    def handle_land(signum, frame):
        """Interrupt: Soft Landing Here"""
        print(f"\n[{args.id}] INTERRUPT: SOFT LANDING INITIATED.", flush=True)
        hal.land()
        # Note: We do NOT sys.exit() here. We let the loop run to manage the descent.

    def emergency_kill(signum, frame):
        """Interrupt: Hard Kill (Ctrl+C)"""
        print(f"\n[{args.id}] INTERRUPT: EMERGENCY KILL (SIGINT).", flush=True)
        hal.land() # Try to command land first
        sys.exit(0)

    # Register Signals (Check for Windows compatibility)
    signal.signal(signal.SIGINT, emergency_kill)
    signal.signal(signal.SIGTERM, emergency_kill)
    
    if hasattr(signal, 'SIGUSR1'):
        signal.signal(signal.SIGUSR1, handle_rtl)
    if hasattr(signal, 'SIGUSR2'):
        signal.signal(signal.SIGUSR2, handle_land)

    # --- COMMS & MISSION ---
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
            # Trigger the handler programmatically
            handle_rtl(None, None)
        elif cmd == 'LAND':
            # Trigger the handler programmatically
            handle_land(None, None)

    comms.sub("fleet/telemetry", lambda m: loop.call_soon_threadsafe(safe_update_telemetry, m))
    comms.sub("fleet/target", lambda m: loop.call_soon_threadsafe(safe_update_target, m))
    comms.sub(f"fleet/{args.id}/command", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))
    comms.sub("fleet/broadcast", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))

    print("[Main] Auto-arming and Starting Mission...", flush=True)
    manager.start() 

    # --- MAIN LOOP ---
    dt = 0.1
    last_print_time = 0.0 

    while True:
        t0 = time.time()
        state = hal.get_state()
        
        # --- Logic ---
        # If we are in LAND mode (from interrupt), check if we touched down
        if state.mode == "LAND":
            alt = -state.position_local[2]
            if alt < 0.2:
                print(f"[{args.id}] Touchdown confirmed. Disarming.", flush=True)
                # In a real script, you might sys.exit() here or wait
                # For sim, we continue but stop printing heavily
        
        if state.mode != "LAND":
             # Only run mission logic if NOT in a landing override
            frame = camera.get_frame()
            detections = detector_manager.process(frame)
            for d in detections:
                cx, cy = d.bbox[0] + d.bbox[2]/2, d.bbox[1] + d.bbox[3]/2
                ang_x = math.atan2(cx - 320, georef.f_px)
                ang_y = math.atan2(cy - 240, georef.f_px)
                lidar_dist = lidar.get_range_at_angle(ang_x, ang_y)
                world_pos = georef.pixel_to_world(cx, cy, state, lidar_range=lidar_dist)
                
                if world_pos is not None and abs(world_pos[2]) < 5.0:
                    ts = time.time()
                    print(f"[{args.id}] [{ts:.3f}] *** CONTACT *** {d.label} at {world_pos} (Conf: {d.confidence:.2f})", flush=True)
                    target_msg = {'t': ts, 'pos': world_pos.tolist(), 'label': d.label, 'conf': d.confidence, 'lidar_range': lidar_dist, 'id': args.id}
                    comms.pub("fleet/target", target_msg)
                    world.targets.update(world_pos, d.label, d.confidence)

            wind_est = world.wind.get_at(state.position_local)
            world.probability.evolve(dt, drift_vector_ms=wind_est)
            manager.check_triggers(state)
            
            raw_cmd = solver.solve(state)
            safe_cmd = safety.filter_velocity(raw_cmd, state)
            hal.send_velocity_command(safe_cmd)
        
        # Telemetry
        telem_packet = {
            'id': args.id, 't': time.time(),
            'pos': state.position_local.tolist(), 'vel': state.velocity.tolist(),
            'hdg': math.degrees(state.heading), 'batt': state.battery,
            'armed': state.armed, 'mode': state.mode, 
            'phase': manager.current_phase_idx, 'posture': manager.current_posture
        }
        comms.pub("fleet/telemetry", telem_packet)

        if time.time() - last_print_time > 2.0:
            last_print_time = time.time()
            print(f"[{args.id}] Phase:{manager.current_phase_idx} | Batt:{state.battery:.0f}% | Mode:{state.mode}", flush=True)

        elapsed = time.time() - t0
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass