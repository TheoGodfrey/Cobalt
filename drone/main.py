"""
Main entry point for COBALT drone client.
Usage: python main.py --id scout_1 --mission missions/mob_search_001.json

FIX for Bug #10: MissionLogger now receives drone_id parameter

REFACTOR: Added hardware_list dependency injection.
REFACTOR: Now loads and merges system_config.yaml and fleet_config.yaml.
"""
import asyncio
import argparse
from pathlib import Path
from typing import List

# Bug #4 fix: Use load_mission_file, not load_mission
from core.g1_mission_definition.loader import load_mission_file
from core.g1_mission_definition.parser import parse_mission_flow
from core.g2_execution_core.mission_controller import MissionController

# Bug #2 fix: Factory functions exist
from core.g3_capability_plugins import get_detector, get_strategy, get_actuator
from core.g3_capability_plugins.detectors.obstacle_detector import ObstacleDetector # <-- NEW

# Bug #3 fix: get_flight_controller factory exists
from core.g4_platform_interface.hal import get_flight_controller
from core.g4_platform_interface.vehicle_state import VehicleState
from core.g4_platform_interface.sensors.lidar import Lidar # <-- NEW

from core.cross_cutting.communication import MqttClient
from core.cross_cutting.safety_monitor import SafetyMonitor

# Bug #6 fix: load_config exists
from core.utils.config_models import load_config

# Bug #10 fix: Import MissionLogger
from core.utils.logger import MissionLogger

async def main():
    parser = argparse.ArgumentParser(
        description='COBALT Drone Mission Controller',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    # ... existing parser arguments ...
    parser.add_argument('--id', required=True, 
                       help='Drone ID (e.g., scout_1, payload_1)')
    parser.add_argument('--mission', required=True, 
                       help='Path to mission JSON file')
    # UPDATED: Default is now system_config.yaml
    parser.add_argument('--config', default='config/system_config.yaml',
                       help='Path to SYSTEM configuration file (default: config/system_config.yaml)')
    parser.add_argument('--log-dir', default='logs',
                       help='Directory for log files (default: logs)')
    parser.add_argument('--max-logs', type=int, default=0,
                       help='Maximum number of log files to keep (0 = unlimited)')
    
    args = parser.parse_args()
    
    # ====================================================================================
    # ... existing logger setup ...
    # ====================================================================================
    logger = MissionLogger(
        log_dir=args.log_dir,
        max_logs=args.max_logs,
        drone_id=args.id  # CRITICAL: Must pass drone_id
    )
    logger.log(f"Starting COBALT drone: {args.id}", "info")
    logger.log(f"Mission file: {args.mission}", "info")
    logger.log(f"System Config file: {args.config}", "info")
    # ====================================================================================
    
    try:
        # 1. Load configuration and mission
        logger.log("Loading configuration...", "info")
        
        # --- NEW: Load and merge both config files ---
        # Load the main system config (network, plugins, etc.)
        system_config = load_config(args.config)
        
        # Find and load the fleet config (assuming it's in the same 'config' dir)
        config_dir = Path(args.config).parent
        fleet_config_path = config_dir / "fleet_config.yaml"
        
        if not fleet_config_path.exists():
            logger.log(f"CRITICAL: fleet_config.yaml not found at {fleet_config_path}", "error")
            raise FileNotFoundError(f"fleet_config.yaml not found at {fleet_config_path}")
            
        logger.log(f"Loading fleet config from {fleet_config_path}...", "info")
        fleet_config_data = load_config(str(fleet_config_path)) # This will be a dict, e.g., {"fleet": {...}}

        # --- NEW: Load separate safety config ---
        # --- FIX: Changed path to load from root /config directory ---
        safety_config_path = config_dir / "safety_config.yaml"
        # --- End of FIX ---
        
        if not Path(safety_config_path).exists():
            logger.log(f"WARNING: {safety_config_path} not found. Safety monitor may not be configured.", "warning")
            safety_config_data = {}
        else:
            logger.log(f"Loading safety config from {safety_config_path}...", "info")
            safety_config_data = load_config(str(safety_config_path))
        # --- End of NEW ---

        # Merge them. The fleet and safety data is top-priority.
        config = {**system_config, **fleet_config_data, **safety_config_data}
        # --- END OF NEW LOADING LOGIC ---
        
        # --- NEW: Get role from fleet_config as single source of truth ---
        fleet_config = config.get('fleet', {})
        drone_config = fleet_config.get(args.id, {})
        
        drone_role = drone_config.get('role')
        if not drone_role:
            logger.log(f"CRITICAL: No 'role' defined for drone '{args.id}' in fleet_config.yaml.", "error")
            raise ValueError(f"No 'role' defined for drone '{args.id}' in fleet_config.yaml.")
        
        # Get all unique, valid roles defined in the fleet config
        valid_roles = list(set(
            d.get('role') for d in fleet_config.values() if d.get('role')
        ))
        logger.log(f"Drone role identified as: '{drone_role}'", "info")
        logger.log(f"All valid fleet roles: {valid_roles}", "info")
        # --- End of FIX ---

        # --- FIX: Pass valid_roles to the parser ---
        logger.log("Loading mission file...", "info")
        mission_data = load_mission_file(Path(args.mission))
        mission_flow = parse_mission_flow(mission_data, valid_roles) # <-- MODIFIED
        logger.log(f"Mission '{mission_flow.mission_id}' loaded successfully", "info")
        
        # --- NEW: Create shared shutdown event ---
        mission_active_event = asyncio.Event()
        mission_active_event.set() # Start in the "running" state
        # --- End of NEW ---
        
        # 2. Create hardware layer
        logger.log(f"Initializing HAL for {args.id}...", "info")
        flight_controller = get_flight_controller(config, args.id)
        vehicle_state = flight_controller.vehicle_state
        
        logger.log("Connecting to hardware...", "info")
        await flight_controller.connect()
        logger.log("Hardware connection established", "info")
        
        # --- NEW: Detect ACTUAL hardware ---
        logger.log("Detecting actual hardware...", "info")
        actual_hardware = await flight_controller.detect_hardware()
        logger.log(f"Actual hardware detected: {actual_hardware}", "info")
        # --- End of NEW ---
        
        # 3. Create communication
        mqtt_config = {
            'host': config.get('network', {}).get('mqtt_broker_host', 'localhost'),
            'port': config.get('network', {}).get('mqtt_broker_port', 1883),
            'user': config.get('network', {}).get('mqtt_user'),
            'password': config.get('network', {}).get('mqtt_pass'),
        }
        
        logger.log(f"Connecting to MQTT broker at {mqtt_config['host']}:{mqtt_config['port']}...", "info")
        mqtt = MqttClient(client_id=args.id, **mqtt_config)
        await mqtt.connect()
        logger.log("MQTT connection established", "info")
        
        # --- FIX 1.2: Re-ordered initialization ---
        
        # --- NEW: Create standalone plugins for SafetyMonitor ---
        obstacle_detector = None
        if "lidar" in actual_hardware:
            try:
                lidar_sensor = flight_controller.get_lidar_sensor(0)
                await lidar_sensor.start() # Start the sensor
                obstacle_config = config.get('detectors', {}).get('obstacle_detector', {})
                obstacle_detector = ObstacleDetector(lidar_sensor, obstacle_config)
                logger.log("Obstacle detector initialized.", "info")
            except Exception as e:
                logger.log(f"Failed to initialize ObstacleDetector: {e}", "error")
        else:
            logger.log("No 'lidar' in detected hardware. Obstacle detector disabled.", "warning")
        # --- End of NEW ---
        
        # 5. Create mission controller FIRST (so mission_state exists)
        logger.log("Initializing mission controller...", "info")
        controller = MissionController(
            mission_flow=mission_flow,
            flight_controller=flight_controller,
            vehicle_state=vehicle_state,
            mqtt=mqtt,
            safety_monitor=None,  # Pass None for now
            config=config,
            hardware_list=actual_hardware,  # <-- MODIFIED: Pass actual hardware
            drone_role=drone_role,
            mission_active_event=mission_active_event # <-- Pass event
        )

        # --- REMOVED P2P Controller Initialization ---
        # controller.p2p_controller = None # This is implicitly handled by the class now

        # 4. Create safety monitor SECOND
        logger.log("Initializing safety monitor...", "info")
        safety_config = config.get('safety', {})
        safety_monitor = SafetyMonitor(
            config=safety_config,
            mqtt=mqtt,
            mission_controller=controller,  # <-- Keep passing full controller
            vehicle_state=vehicle_state,
            mission_active_event=mission_active_event, # <-- Pass event
            flight_controller=flight_controller, # <-- NEW
            obstacle_detector=obstacle_detector  # <-- NEW (will be None if disabled)
        )
        
        # Now, link the safety_monitor back to the controller
        controller.safety_monitor = safety_monitor
        
        # --- End of FIX 1.2 ---
        
        logger.log("=" * 70, "info")
        logger.log(f"MISSION START: {mission_flow.mission_id}", "info")
        logger.log("=" * 70, "info")
        
        # 6. Run mission
        await asyncio.gather(
            controller.execute_mission(),
            safety_monitor.monitor_loop()
        )
        
        logger.log("=" * 70, "info")
        logger.log("MISSION COMPLETE", "info")
        logger.log("=" * 70, "info")
        
        # ... existing summary logging ...
        summary = {
            "Mission ID": mission_flow.mission_id,
            "Drone ID": args.id,
            "Final State": controller.mission_state.current.name,
            "Final Battery": f"{vehicle_state.battery_percent:.1f}%",
            "Mission Duration": "N/A"  # Would calculate from timestamps
        }
        logger.log_summary(summary)
        
    except KeyboardInterrupt:
        logger.log("Mission interrupted by user", "warning")
        print("\n[Main] Shutting down gracefully...")
        if 'mission_active_event' in locals():
            mission_active_event.clear() # Signal loops to exit
        
    except Exception as e:
        logger.log(f"CRITICAL ERROR: {e}", "error")
        import traceback
        logger.log(traceback.format_exc(), "error")
        if 'mission_active_event' in locals():
            mission_active_event.clear() # Signal loops to exit
        raise
        
    finally:
        # ... existing cleanup ...
        try:
            if 'mqtt' in locals() and mqtt.is_connected():
                await mqtt.disconnect()
            if 'flight_controller' in locals():
                await flight_controller.disarm()
            if 'lidar_sensor' in locals() and lidar_sensor: # <-- NEW
                await lidar_sensor.stop()
            logger.log("Shutdown complete", "info")
            print(f"\n[Main] Logs saved to: {logger.get_log_path()}")
        except Exception as e:
            print(f"[Main] Error during cleanup: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Main] Interrupted")
    except Exception as e:
        print(f"\n[Main] Fatal error: {e}")
        import sys
        sys.exit(1)