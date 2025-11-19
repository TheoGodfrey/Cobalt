import argparse
import yaml
import time
import asyncio
import numpy as np
from cobalt.core.state.world_state import WorldState
from cobalt.core.math.solver import ProbabilisticSolver
from cobalt.core.mission.manager import MissionManager
from cobalt.core.comms import Comms
from cobalt.drone.safety.monitor import SafetyMonitor
from cobalt.drone.hardware.hal import MockHAL, MavlinkHAL

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
    with open("missions/hunter_killer.yaml") as f: mission = yaml.safe_load(f)

    # 2. Init
    world = WorldState(fleet['world'])
    solver = ProbabilisticSolver()
    safety = SafetyMonitor()
    hal = MavlinkHAL(my_conf['connection']) if args.hal == "mavlink" else MockHAL()
    comms = Comms(args.id)
    
    # Manager initialized without a role
    manager = MissionManager(mission, world, solver, safety)

    # 3. Comms Setup
    hal.connect()
    comms.connect()

    # Listen for POSTURE updates from Hub (Dynamic Assignment)
    comms.sub(f"fleet/{args.id}/posture", lambda msg: manager.set_posture(msg['posture']))
    
    def on_target(msg):
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        world.probability.update_from_sensor(msg['pos'][0], msg['pos'][1], 10, msg['conf'])
    comms.sub("fleet/target", on_target)

    # 4. Start
    manager.start()

    # 5. Loop
    dt = 0.1
    while True:
        t0 = time.time()
        state = hal.get_state()
        
        comms.pub("fleet/telemetry", {'id': args.id, 'pos': state.pos.tolist()})
        
        world.probability.evolve(dt)
        manager.check_triggers()
        
        raw = solver.solve(state)
        safe = safety.filter_velocity(raw, state)
        hal.apply_velocity(safe)
        
        # Print posture instead of role
        print(f"[{manager.current_posture}] Pos: {state.pos.astype(int)} | Cmd: {safe.astype(int)}")
        await asyncio.sleep(max(0, dt - (time.time()-t0)))

if __name__ == "__main__":
    asyncio.run(main())