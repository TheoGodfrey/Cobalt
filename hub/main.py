import yaml
import time
import numpy as np
from cobalt.core.state.world_state import WorldState
from cobalt.core.comms import Comms

def main():
    print("[Hub] Starting Dynamic Fleet Orchestrator...")
    
    # 1. Config
    with open("config/fleet_config.yaml") as f: fleet_conf = yaml.safe_load(f)
    
    # 2. State
    world = WorldState(fleet_conf['world'])
    comms = Comms("hub_master")
    comms.connect()
    
    fleet_positions = {}
    
    # 3. Callbacks
    def on_telem(msg):
        fleet_positions[msg['id']] = np.array(msg['pos'])
    
    def on_target(msg):
        world.targets.update(msg['pos'], msg['label'], msg['conf'])
        comms.pub("fleet/target", msg)
        
        # --- DYNAMIC POSTURE ASSIGNMENT ---
        # 1. Identify the best target
        best_target = world.targets.get_best_target()
        if not best_target or best_target.confidence < 0.8:
            return

        # 2. Calculate scores (Distance to target)
        scores = {}
        for did, pos in fleet_positions.items():
            dist = np.linalg.norm(pos - best_target.position)
            scores[did] = dist

        # 3. Assign Postures
        # Closest drone becomes 'primary_interceptor'
        # All others become 'supporting_tracker'
        if not scores: return
        
        closest_drone = min(scores, key=scores.get)
        
        for did in fleet_positions:
            if did == closest_drone:
                posture = "primary_interceptor"
            else:
                posture = "supporting_tracker"
                
            print(f"[Hub] Assigning {did} -> {posture}")
            # Send the posture assignment to the drone
            comms.pub(f"fleet/{did}/posture", {'posture': posture})

    comms.sub("fleet/telemetry", on_telem)
    comms.sub("fleet/target", on_target)

    # 4. Loop
    while True:
        time.sleep(1.0)

if __name__ == "__main__":
    main()