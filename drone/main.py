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

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--mission", default="missions/MOB_mission.yaml")
    parser.add_argument("--hal", default="mock")
    args = parser.parse_args()

    print(f"[Main] Starting COBALT Drone: {args.id}")

    base_path = Path(__file__).parent.parent
    
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
        print(f"[Main] CRITICAL: Mission file {args.mission} not found.")
        return

    world = WorldState(fleet_conf.get('world'))
    world.my_id = args.id 
    world.fleet_positions = {} 

    solver = ProbabilisticSolver(config={'max_speed_xy': 20.0})
    safety = SafetyMonitor(world, config={'min_altitude': 5.0})
    
    if args.hal == "mavlink":
        hal = MavlinkHAL(my_conf['connection'])
    else:
        hal = MockHAL() 

    # --- Signal Handlers ---
    def emergency_shutdown(signum, frame):
        print(f"\n[{args.id}] CAUGHT SIGNAL {signum}. INITIATING EMERGENCY LANDING.")
        hal.land()
        sys.exit(0)

    signal.signal(signal.SIGINT, emergency_shutdown)
    signal.signal(signal.SIGTERM, emergency_shutdown)
    # -----------------------

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
        # --- NEW: Safety Commands ---
        elif cmd == 'RTL':
            print(f"[{args.id}] Received Global RTL Command.")
            hal.return_to_launch()
        elif cmd == 'LAND':
            print(f"[{args.id}] Received Global LAND Command.")
            hal.land()

    comms.sub("fleet/telemetry", lambda m: loop.call_soon_threadsafe(safe_update_telemetry, m))
    comms.sub("fleet/target", lambda m: loop.call_soon_threadsafe(safe_update_target, m))
    # Listen to specific commands AND broadcast fleet commands
    comms.sub(f"fleet/{args.id}/command", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))
    comms.sub("fleet/broadcast", lambda m: loop.call_soon_threadsafe(safe_mission_command, m))

    print("[Main] Auto-arming and Starting Mission...")
    manager.start() 

    dt = 0.1
    while True:
        t0 = time.time()
        state = hal.get_state()
        
        altitude = -state.position_local[2]
        if altitude > 1.0: 
            world.probability.update_from_sensor(
                x=state.position_local[0], y=state.position_local[1],
                z_drone=altitude, fov_angle_deg=60.0
            )
        
        wind_est = world.wind.get_at(state.position_local)
        world.probability.evolve(dt, drift_vector_ms=wind_est)

        manager.check_triggers(state)

        raw_cmd = solver.solve(state)
        solver_debug = solver.get_debug_info() 
        
        safe_cmd = safety.filter_velocity(raw_cmd, state)
        hal.send_velocity_command(safe_cmd)
        
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

        if int(time.time()) % 2 == 0 and int(time.time()*10) % 10 == 0:
             print(f"[{args.id}] Phase:{manager.current_phase_idx} | Batt:{state.battery:.0f}% | Mode:{state.mode}")

        elapsed = time.time() - t0
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)

if __name__ == "__main__":
    asyncio.run(main())