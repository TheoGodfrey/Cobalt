"""
Main entry point for COBALT drone client.
Usage: python main.py --id scout_1 --mission missions/mob_search_001.json

FIX for Bug #10: MissionLogger now receives drone_id parameter

REFACTOR: Added hardware_list dependency injection.
"""
import asyncio
import argparse
from pathlib import Path
from typing import List  # NEW: For hardware_list

# Bug #4 fix: Use load_mission_file, not load_mission
from core.g1_mission_definition.loader import load_mission_file
from core.g1_mission_definition.parser import parse_mission_flow
from core.g2_execution_core.mission_controller import MissionController

# Bug #2 fix: Factory functions exist
from core.g3_capability_plugins import get_detector, get_strategy, get_actuator

# Bug #3 fix: get_flight_controller factory exists
from core.g4_platform_interface.hal import get_flight_controller
from core.g4_platform_interface.vehicle_state import VehicleState

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
    parser.add_argument('--config', default='config/system_config.yaml',
                       help='Path to configuration file (default: config/system_config.yaml)')
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
    logger.log(f"Config file: {args.config}", "info")
    # ====================================================================================
    
    try:
        # 1. Load configuration and mission
        logger.log("Loading configuration...", "info")
        config = load_config(args.config)
        
        logger.log("Loading mission file...", "info")
        mission_data = load_mission_file(Path(args.mission))
        mission_flow = parse_mission_flow(mission_data)
        logger.log(f"Mission '{mission_flow.mission_id}' loaded successfully", "info")
        
        # --- NEW: Hardware-Aware Refactor ---
        # Look up this drone's specific config from the fleet
        fleet_config = config.get('fleet', {})
        drone_config = fleet_config.get(args.id, {})
        
        # Extract the hardware list. This assumes a config structure like:
        # fleet:
        #   scout_1:
        #     role: "scout"
        #     hal: "simulated"
        #     hardware: ["gps", "camera_thermal", "dropper_mechanism"]
        hardware_list = drone_config.get('hardware', [])
        if not hardware_list:
            logger.log(f"No 'hardware' list found for '{args.id}' in config. "
                         f"Hardware validation may fail.", "warning")
        else:
            logger.log(f"Found hardware: {hardware_list}", "info")
        # --- End of NEW ---
        
        # 2. Create hardware layer
        logger.log(f"Initializing HAL for {args.id}...", "info")
        flight_controller = get_flight_controller(config, args.id)
        vehicle_state = flight_controller.vehicle_state
        
        logger.log("Connecting to hardware...", "info")
        await flight_controller.connect()
        logger.log("Hardware connection established", "info")
        
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
        
        # 5. Create mission controller FIRST (so mission_state exists)
        logger.log("Initializing mission controller...", "info")
        controller = MissionController(
            mission_flow=mission_flow,
            flight_controller=flight_controller,
            vehicle_state=vehicle_state,
            mqtt=mqtt,
            safety_monitor=None,  # Pass None for now
            config=config,
            hardware_list=hardware_list  # NEW: Pass hardware list
        )
        
        # 4. Create safety monitor SECOND
        logger.log("Initializing safety monitor...", "info")
        safety_config = config.get('safety', {})
        safety_monitor = SafetyMonitor(
            config=safety_config,
            mqtt=mqtt,
            mission_state=controller.mission_state,  # <-- Pass the *existing* instance
            vehicle_state=vehicle_state
        )
        
        # Now, link the safety_monitor back to the controller
        controller.safety_monitor = safety_monitor
        
        # The old line below is no longer needed, as it's done in the constructor
        # safety_monitor.mission_state = controller.mission_state
        
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
        
    except Exception as e:
        logger.log(f"CRITICAL ERROR: {e}", "error")
        import traceback
        logger.log(traceback.format_exc(), "error")
        raise
        
    finally:
        # ... existing cleanup ...
        try:
            if 'mqtt' in locals():
                await mqtt.disconnect()
            if 'flight_controller' in locals():
                await flight_controller.disarm()
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
