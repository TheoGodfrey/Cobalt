import yaml
import time
import sys
import argparse
import subprocess
import signal
import select
import numpy as np
from pathlib import Path

# Import Core
from core.state.world_state import WorldState
from core.comms import Comms

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", default="missions/MOB_mission.yaml", help="Path to mission file")
    parser.add_argument("--mode", default="sim", choices=["sim", "deploy"], help="Run mode")
    args = parser.parse_args()

    print("[Hub] Starting Dynamic Fleet Orchestrator...")
    print("[Hub] Interactive Mode: [r] RTL | [l] LAND | [q] QUIT")
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / "config/fleet_config.yaml"
    
    try:
        with open(config_path) as f: fleet_conf = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"[Hub] Critical: Config not found at {config_path}")
        return

    drone_processes = []
    
    def cleanup(signum, frame):
        print("\n[Hub] Shutting down fleet...")
        for p in drone_processes:
            p.terminate()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f"[Hub] Spawning fleet from {config_path}...")
    
    for drone_id, drone_data in fleet_conf.get('fleet', {}).items():
        print(f"   >>> Launching {drone_id} ({drone_data.get('type', 'unknown')})...")
        cmd = [
            sys.executable, "drone/main.py", 
            "--id", drone_id, "--mission", args.mission,
            "--hal", "mock" if args.mode == "sim" else "mavlink"
        ]
        p = subprocess.Popen(cmd)
        drone_processes.append(p)

    world = WorldState(fleet_conf['world'])
    comms = Comms("hub_master")
    comms.connect()
    
    fleet_positions = {}
    
    def on_telem(msg):
        fleet_positions[msg['id']] = np.array(msg['pos'])
    
    def on_target(msg):
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        comms.pub("fleet/target", msg)
        
        best_target = world.targets.get_best_target()
        if not best_target or best_target.confidence < 0.8: return

        scores = {}
        for did, pos in fleet_positions.items():
            dist = np.linalg.norm(pos - best_target.position)
            scores[did] = dist

        if not scores: return
        closest = min(scores, key=scores.get)
        
        # In a real mission, we might send posture commands here
        # But we avoid spamming logic loop for now

    comms.sub("fleet/telemetry", on_telem)
    comms.sub("fleet/target", on_target)

    print("[Hub] Fleet Airborne. Monitoring...")

    try:
        while True:
            # 1. Check for Keyboard Input (Non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'r':
                    print("[Hub] >>> BROADCAST: RETURN TO LAUNCH (RTL)")
                    comms.pub("fleet/broadcast", {'command': 'RTL'})
                elif cmd == 'l':
                    print("[Hub] >>> BROADCAST: LAND IMMEDIATELY")
                    comms.pub("fleet/broadcast", {'command': 'LAND'})
                elif cmd == 'q':
                    raise KeyboardInterrupt
            
            # 2. Monitor Processes
            for p in drone_processes:
                if p.poll() is not None:
                    print(f"[Hub] Warning: A drone process died (Exit Code: {p.returncode})")
                    drone_processes.remove(p)
            
            time.sleep(0.1) # Fast loop for responsive input
            
    except KeyboardInterrupt:
        cleanup(None, None)

if __name__ == "__main__":
    main()