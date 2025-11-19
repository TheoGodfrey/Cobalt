import yaml
import time
import sys
import argparse
import subprocess
import signal
import numpy as np
from pathlib import Path

# Cross-Platform Input Handling
try:
    import msvcrt # Windows
    def kbhit(): return msvcrt.kbhit()
    def getch(): return msvcrt.getch().decode('utf-8').lower()
    print("[Hub] Input Mode: Windows (msvcrt)")
except ImportError:
    import select # Mac/Linux
    import tty, termios
    def kbhit():
        dr,dw,de = select.select([sys.stdin], [], [], 0)
        return dr != []
    def getch():
        return sys.stdin.read(1).lower()
    print("[Hub] Input Mode: Unix (select)")

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
    print("[Hub] COMMANDS: [r] Return to Base | [l] Land Now | [q] Quit", flush=True)
    
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
        
        cmd = [
            sys.executable, "-u", "drone/main.py", 
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
    
    last_print = 0
    def on_target(msg):
        nonlocal last_print
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        comms.pub("fleet/target", msg)
        
        if time.time() - last_print > 2.0:
            ts = msg.get('t', time.time())
            print(f"[Hub] [{ts:.3f}] TARGET: {msg['label']} (Conf: {msg['conf']:.2f})", flush=True)
            last_print = time.time()

    comms.sub("fleet/telemetry", on_telem)
    comms.sub("fleet/target", on_target)

    print("[Hub] Fleet Airborne. Monitoring...", flush=True)

    try:
        while True:
            # 1. Check Process Health
            for p in drone_processes:
                if p.poll() is not None:
                    print(f"[Hub] CRITICAL: A drone process died unexpectedly (Exit Code: {p.returncode})", flush=True)
                    drone_processes.remove(p)

            # 2. Cross-Platform Input Check
            if kbhit():
                cmd = getch()
                if cmd == 'q':
                    raise KeyboardInterrupt
                elif cmd == 'r':
                    print("\n[Hub] >>> BROADCAST INTERRUPT: RETURN TO BASE (RTL) <<<")
                    comms.pub("fleet/broadcast", {'command': 'RTL', 't': time.time()})
                elif cmd == 'l':
                    print("\n[Hub] >>> BROADCAST INTERRUPT: SOFT LANDING (LAND) <<<")
                    comms.pub("fleet/broadcast", {'command': 'LAND', 't': time.time()})

            time.sleep(0.1)
            
    except KeyboardInterrupt:
        cleanup(None, None)

if __name__ == "__main__":
    main()