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

# Import Equations
from core.maths.equations.rescue import UnifiedRescue
from core.maths.equations.constraint import ObstacleRepulsion
from core.maths.equations.swarm import SwarmRepulsion

# Import Detectors
from core.detectors.thermal.manager import DetectorManager # Assuming this exists or uses base
from core.detectors.thermal.base import BaseDetector # Fallback
# (Detectors imported as before...)

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--hal", default="mock")
    args = parser.parse_args()

    print(f"[Main] Starting COBALT Drone: {args.id}")

    # 1. Config
    try:
        with open("config/fleet_config.yaml") as f: fleet = yaml.safe_load(f)
        # Use safe get or default for robustness
        my_conf = fleet.get('fleet', {}).get(args.id, {'connection': 'udp:127.0.0.1:14550'})
    except FileNotFoundError:
        print("Config file not found, using defaults.")
        fleet = {'world': {}}
        my_conf = {'connection': 'udp:127.0.0.1:14550'}

    # 2. Init Core Components
    world = WorldState(fleet.get('world'))
    
    # Inject self ID into world (for swarm logic filtering)
    world.my_id = args.id 
    # Also initialize a fleet_positions dict for swarm tracking
    world.fleet_positions = {} 

    solver = ProbabilisticSolver(config={'max_speed_xy': 15.0})
    safety = SafetyMonitor(config={'min_altitude': 5.0})
    
    if args.hal == "mavlink":
        hal = MavlinkHAL(my_conf['connection'])
    else:
        hal = MockHAL() # Ensure MockHAL is updated to support 3D
        
    comms = Comms(args.id)

    # 3. Initialize Equations
    # We manually set up the equation stack here for the MVP.
    # In a full version, MissionManager would swap these based on phase.
    
    # A. Mission (Attraction to Probability)
    eq_rescue = UnifiedRescue(world, config={'cruise_speed': 12.0, 'risk_theta': 0.1})
    
    # B. Constraints (Repulsion from Obstacles)
    eq_constraint = ObstacleRepulsion(world, config={'safety_margin': 10.0})
    
    # C. Swarm (Repulsion from Neighbors)
    eq_swarm = SwarmRepulsion(world, config={'drone_id': args.id, 'separation': 20.0})

    # Load them into Solver
    solver.set_equation(eq_rescue) # Primary
    solver.add_equation(eq_constraint)
    solver.add_equation(eq_swarm)

    # 4. Comms Setup
    await hal.connect() # HAL connect is async
    comms.connect()

    # Telemetry Listener: Update world.fleet_positions
    def on_telemetry(msg):
        if msg['id'] != args.id:
            # Store neighbor position [x, y, z]
            world.fleet_positions[msg['id']] = np.array(msg['pos'])
            
    comms.sub("fleet/telemetry", on_telemetry)

    # Target Listener: Update Probability Grid
    def on_target(msg):
        world.update_sensor_detection(msg)
    comms.sub("fleet/target", on_target)

    # 5. Main Control Loop (10Hz)
    print("[Main] Entering Control Loop...")
    dt = 0.1
    
    # Arm and Takeoff (Simple Sequence for MVP)
    # In a real mission, this is handled by the MissionManager state machine
    if args.hal == "mock":
        print("[Main] Auto-arming and taking off (Mock)...")
        # hal.arm() 
        # hal.takeoff(10.0) 

    while True:
        t0 = time.time()
        
        # --- A. Sense ---
        state = hal.get_state() # DroneState(pos, vel, batt, etc.)
        
        # --- B. Update World Belief ---
        # 1. Self-Update: Clear the cone of vision (Negative Information)
        # We assume we are looking down-ish.
        # Convert NED z to Altitude h
        altitude = -state.position_local[2]
        if altitude > 1.0: # Only update if airborne
            world.probability.update_from_sensor(
                x=state.position_local[0],
                y=state.position_local[1],
                z_drone=altitude,
                fov_angle_deg=60.0,
                confidence=0.4 # Reduce prob (clear the area)
            )
            
        # 2. Wind Update: Add observation
        # We estimate wind by comparing (GroundSpeed - AirSpeed). 
        # For MVP Mock, we just assume we know it or skip update.
        # world.wind.update(...)
        
        # --- C. Evolve World Physics ---
        world.probability.evolve(dt, drift_vector_ms=world.wind.get_at(state.position_local))

        # --- D. Solve Control Problem ---
        # Get raw velocity command from gradient descent
        raw_cmd = solver.solve(state)
        
        # Apply Safety Limits (Hard Deck, Speed Cap)
        safe_cmd = safety.filter_velocity(raw_cmd, state)
        
        # --- E. Actuate ---
        hal.send_velocity_command(safe_cmd)
        
        # --- F. Sync ---
        # Broadcast my state to the swarm
        comms.pub("fleet/telemetry", {
            'id': args.id, 
            'pos': state.position_local.tolist(),
            'vel': state.velocity.tolist()
        })

        # Debug Print (1Hz)
        if int(time.time()) % 1 == 0:
             # debug = solver.get_debug_info()
             print(f"Pos: {state.position_local.astype(int)} | Cmd: {safe_cmd.astype(int)}")

        # Maintain Loop Rate
        elapsed = time.time() - t0
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)

if __name__ == "__main__":
    asyncio.run(main())