import argparse
import yaml
import sys
import os
import subprocess
from pathlib import Path

def load_config(path):
    """Safely loads a YAML configuration file."""
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"[Error] File not found: {path}")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"[Error] Invalid YAML in {path}: {e}")
        sys.exit(1)

def validate_mission_requirements(fleet_conf, mission_conf):
    """
    Checks if the fleet meets the mission's drone count and type requirements.
    """
    print("[Init] Validating Fleet Capabilities...")
    
    # 1. Get Requirements (Default to 1 drone if not specified)
    requirements = mission_conf.get('requirements', {'min_drones': 1})
    min_drones = requirements.get('min_drones', 1)
    required_types = requirements.get('required_types', {})

    # 2. Analyze Fleet
    fleet = fleet_conf.get('fleet', {})
    total_drones = len(fleet)
    
    # Count types available in fleet (default to 'standard' if type is missing)
    available_types = {}
    for drone_id, specs in fleet.items():
        d_type = specs.get('type', 'standard')
        available_types[d_type] = available_types.get(d_type, 0) + 1

    # 3. Validation Logic
    errors = []

    # Check Total Count
    if total_drones < min_drones:
        errors.append(f"Insufficient drone count: Need {min_drones}, have {total_drones}.")

    # Check Specific Types
    for req_type, req_count in required_types.items():
        have = available_types.get(req_type, 0)
        if have < req_count:
            errors.append(f"Missing drone type '{req_type}': Need {req_count}, have {have}.")

    # 4. Result
    if errors:
        print("[Validation Failed]")
        for e in errors:
            print(f" - {e}")
        return False
    
    print(f"[Validation Passed] Fleet ready with {total_drones} drones.")
    return True

def main():
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(
        description="Cobalt: Autonomous MOB Rescue System Orchestrator"
    )
    
    parser.add_argument(
        "--mission", 
        type=str, 
        default="missions/MOB_mission.yaml",
        help="Path to the mission definition YAML file"
    )
    
    parser.add_argument(
        "--fleet", 
        type=str, 
        default="config/fleet_config.yaml",
        help="Path to the fleet configuration YAML file"
    )

    parser.add_argument(
        "--mode",
        choices=["validate", "sim", "deploy"],
        default="sim",
        help="Execution mode: 'validate' (check only), 'sim' (mock hardware), 'deploy' (real hardware)"
    )

    args = parser.parse_args()

    # --- Load Configurations ---
    print(f"[Main] Loading Mission: {args.mission}")
    mission = load_config(args.mission)
    
    print(f"[Main] Loading Fleet: {args.fleet}")
    fleet = load_config(args.fleet)

    # --- Validate ---
    if not validate_mission_requirements(fleet, mission):
        print("[Main] Aborting Launch due to configuration errors.")
        sys.exit(1)

    if args.mode == "validate":
        print("[Main] Validation complete. Exiting.")
        sys.exit(0)

    # --- Execution ---
    print(f"[Main] Launching Operation in {args.mode.upper()} mode...")
    
    # Example: Launching the Hub process
    # In a real system, you might use subprocess to spawn the Hub and Drones
    try:
        # Using the hub/main.py entry point we saw in your files
        subprocess.run([sys.executable, "hub/main.py"], check=True)
    except KeyboardInterrupt:
        print("\n[Main] Mission Cancelled by User.")
    except subprocess.CalledProcessError as e:
        print(f"[Main] Hub crashed with error code {e.returncode}")

if __name__ == "__main__":
    main()