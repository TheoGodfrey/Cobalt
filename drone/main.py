import argparse
import yaml
import time
import asyncio
import numpy as np
from pathlib import Path

# Import Core
from core.state.worldstate import WorldState
from core.maths.solver import ProbabilisticSolver
from core.mission.manager import MissionManager
from core.comms import Comms
from drone.safety.safety_monitor import SafetyMonitor
from drone.hardware.hal import MockHAL, MavlinkHAL

# Import Detectors
from core.detectors.thermal.manager import DetectorManager
from core.detectors.thermal.absolute import AbsoluteThermalDetector
from core.detectors.thermal.edge import ThermalEdgeDetector
from core.detectors.thermal.statistical import StatisticalAnomalyDetector

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--hal", default="mock")
    # REMOVED --role argument
    args = parser.parse_args()

    print(f"[Main] Starting COBALT Drone: {args.id}")

    # 1. Config
    with open("config/fleet_config.yaml") as f: fleet = yaml.safe_load(f)
    my_conf = fleet['fleet'][args.id]
    
    # Load mission (defaulting to hunter_killer for MVP)
    mission_file = "missions/hunter_killer.yaml"
    print(f"[Main] Loading mission: {mission_file}")
    with open(mission_file) as f: mission = yaml.safe_load(f)

    # 2. Init Core Components
    world = WorldState(fleet['world'])
    solver = ProbabilisticSolver()
    safety = SafetyMonitor()
    hal = MavlinkHAL(my_conf['connection']) if args.hal == "mavlink" else MockHAL()
    comms = Comms(args.id)
    
    # Manager initialized without a role
    manager = MissionManager(mission, world, solver, safety)

    # 3. Init Detectors (Detectors Galore!)
    detector_mgr = DetectorManager()
    # In a real app, these configs would come from fleet_config.yaml
    detector_mgr.add_detector(AbsoluteThermalDetector(threshold_temp=38.0))
    detector_mgr.add_detector(ThermalEdgeDetector())
    detector_mgr.add_detector(StatisticalAnomalyDetector(sigma=3.0))
    print("[Main] Detectors initialized.")

    # 4. Comms Setup
    hal.connect()
    comms.connect()

    # Listen for POSTURE updates from Hub (Dynamic Assignment)
    comms.sub(f"fleet/{args.id}/posture", lambda msg: manager.set_posture(msg['posture']))
    
    def on_target(msg):
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        world.probability.update_from_sensor(msg['pos'][0], msg['pos'][1], 10, msg['conf'])
    comms.sub("fleet/target", on_target)

    # 5. Start Mission
    manager.start()

    # 6. The Main Loop (10Hz)
    dt = 0.1
    while True:
        t0 = time.time()
        
        # A. Sense
        state = hal.get_state()
        
        # In a real implementation, HAL provides the camera frame
        # frame = hal.get_thermal_frame() 
        # For MVP/Mock, we simulate a frame (e.g. zeros or noise)
        # frame = np.zeros((480, 640)) 
        
        # detections = detector_mgr.process(frame)
        
        # Mocking detections for MVP flow:
        detections = [] 
        
        # B. Update World State (The "Look" Logic)
        # Calculate where the camera is looking (center of FOV on ground)
        # Simplified: Assume looking directly below for MVP
        view_x, view_y = state.position_local[0], state.position_local[1]
        
        if detections:
            for d in detections:
                # Positive Update: Found something!
                # In real app: Convert pixel bbox to relative meters
                rel_pos = state.position_local # Placeholder
                
                world.update_sensor_detection({
                    'pos': rel_pos, 
                    'label': d.label, 
                    'confidence': d.confidence,
                    'radius': 10.0
                })
                print(f"[Detection] Found {d.label}!")
        else:
            # Negative Update: Saw nothing, clear the probability map
            # This is critical for the "Search" phase to progress
            world.probability.update_from_sensor(
                x=view_x, 
                y=view_y, 
                radius=20.0, # FOV Radius
                confidence=0.4 # < 0.5 reduces probability (clears area)
            )

        # C. Sync
        comms.pub("fleet/telemetry", {'id': args.id, 'pos': state.position_local.tolist()})
        
        # D. Evolve
        world.probability.evolve(dt)
        manager.check_triggers()
        
        # E. Solve
        raw = solver.solve(state)
        safe = safety.filter_velocity(raw, state)
        
        # F. Act
        hal.send_velocity_command(safe)
        
        # Print status
        print(f"[{manager.current_posture}] Pos: {state.position_local.astype(int)} | Cmd: {safe.astype(int)}")
        
        await asyncio.sleep(max(0, dt - (time.time()-t0)))

if __name__ == "__main__":
    asyncio.run(main())