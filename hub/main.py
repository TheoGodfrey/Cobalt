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

    print("[Hub] Starting Dynamic Fleet Orchestrator...", flush=True)
    print("[Hub] Interactive Mode: [r] RTL | [l] LAND | [q] QUIT", flush=True)
    
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

    print(f"[Hub] Spawning fleet from {config_path}...", flush=True)
    
    for drone_id, drone_data in fleet_conf.get('fleet', {}).items():
        print(f"   >>> Launching {drone_id} ({drone_data.get('type', 'unknown')})...", flush=True)
        cmd = [
            sys.executable, "-u", "drone/main.py",  # <--- ADDED "-u" HERE
            "--id", drone_id, "--mission", args.mission,
            "--hal", "mock" if args.mode == "sim" else "mavlink"
        ]
        # Pass stdout/stderr to parent so we see it in the same terminal
        p = subprocess.Popen(cmd)
        drone_processes.append(p)

    world = WorldState(fleet_conf['world'])
    comms = Comms("hub_master")
    comms.connect()
    
    fleet_positions = {}
    
    def on_telem(msg):
        fleet_positions[msg['id']] = np.array(msg['pos'])
    
    def on_target(msg):
        # Print explicit confirmation when Hub receives a target
        print(f"[Hub] TARGET RECEIVED: {msg['label']} from {msg['id']}", flush=True)
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        comms.pub("fleet/target", msg)

    comms.sub("fleet/telemetry", on_telem)
    comms.sub("fleet/target", on_target)

    print("[Hub] Fleet Airborne. Monitoring...", flush=True)

    try:
        while True:
            # 1. Check for Keyboard Input (Non-blocking, Unix only)
            # Wrap in try-except for Windows compatibility
            try:
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
            except (OSError, ValueError):
                pass # select() fails on Windows non-sockets, ignore
            
            # 2. Monitor Processes
            for p in drone_processes:
                if p.poll() is not None:
                    print(f"[Hub] Warning: A drone process died (Exit Code: {p.returncode})")
                    drone_processes.remove(p)
            
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        cleanup(None, None)

if __name__ == "__main__":
    main()