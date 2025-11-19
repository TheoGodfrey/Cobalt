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
    # Force unbuffered stdout explicitly
    sys.stdout.reconfigure(line_buffering=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", default="missions/MOB_mission.yaml", help="Path to mission file")
    parser.add_argument("--mode", default="sim", choices=["sim", "deploy"], help="Run mode")
    args = parser.parse_args()

    print("[Hub] Starting Dynamic Fleet Orchestrator...", flush=True)
    
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
        print(f"   >>> Launching {drone_id}...", flush=True)
        
        # CRITICAL: We pass "-u" to the drone python process too
        cmd = [
            sys.executable, "-u", "drone/main.py", 
            "--id", drone_id, "--mission", args.mission,
            "--hal", "mock" if args.mode == "sim" else "mavlink"
        ]
        
        # We do NOT use PIPE for stdout, so it flows directly to your terminal
        p = subprocess.Popen(cmd)
        drone_processes.append(p)

    world = WorldState(fleet_conf['world'])
    comms = Comms("hub_master")
    comms.connect()
    
    fleet_positions = {}
    
    def on_telem(msg):
        fleet_positions[msg['id']] = np.array(msg['pos'])
    
    def on_target(msg):
        print(f"[Hub] TARGET RECEIVED: {msg['label']} from {msg['id']}", flush=True)
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        comms.pub("fleet/target", msg)

    comms.sub("fleet/telemetry", on_telem)
    comms.sub("fleet/target", on_target)

    print("[Hub] Fleet Airborne. Monitoring... (Press 'q' to quit)", flush=True)

    try:
        while True:
            # Check for crashed drones
            for p in drone_processes:
                if p.poll() is not None:
                    print(f"[Hub] CRITICAL: A drone process died unexpectedly (Exit Code: {p.returncode})", flush=True)
                    print("      -> Check if you are missing files (camera.py, lidar.py) or libraries.", flush=True)
                    drone_processes.remove(p)

            # Simple input check (Unix/Mac)
            try:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    cmd = sys.stdin.readline().strip().lower()
                    if cmd == 'q': raise KeyboardInterrupt
            except: pass

            time.sleep(0.1)
            
    except KeyboardInterrupt:
        cleanup(None, None)

if __name__ == "__main__":
    main()