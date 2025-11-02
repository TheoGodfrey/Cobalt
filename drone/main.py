"""
Main entry point for COBALT drone client.
Usage: python main.py --id scout_1 --mission missions/mob_search_001.json
"""
import asyncio
import argparse
from pathlib import Path

# BUG #4: Function is load_mission_file, not load_mission
from core.g1_mission_definition.loader import load_mission
from core.g2_execution_core.mission_controller import MissionController
# BUG #5: These factory functions are not defined
from core.g3_capability_plugins import get_detector, get_strategy, get_actuator
from core.g4_platform_interface.hal import get_flight_controller
from core.g4_platform_interface.vehicle_state import VehicleState
from core.cross_cutting.communication import MqttClient
from core.cross_cutting.safety_monitor import SafetyMonitor
# BUG #6: load_config is not defined in the empty config_models.py
from core.utils.config_models import load_config

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', required=True, help='Drone ID (scout_1, payload_1, etc.)')
    parser.add_argument('--mission', required=True, help='Path to mission JSON')
    args = parser.parse_args()
    
    # 1. Load configuration and mission
    config = load_config('config/mission_config.yaml')
    mission_flow = load_mission(args.mission)
    
    # 2. Create hardware layer
    flight_controller = get_flight_controller(config.drone_type, args.id)
    vehicle_state = VehicleState()
    
    # 3. Create communication
    mqtt = MqttClient(config.mqtt, client_id=args.id)
    await mqtt.connect()
    
    # 4. Create safety monitor (cross-cutting)
    safety_monitor = SafetyMonitor(config.safety, mqtt)
    
    # 5. Create mission controller (generic orchestrator)
    controller = MissionController(
        mission_flow=mission_flow,
        flight_controller=flight_controller,
        vehicle_state=vehicle_state,
        mqtt=mqtt,
        safety_monitor=safety_monitor,
        config=config
    )
    
    # 6. Run mission
    await asyncio.gather(
        controller.execute_mission(),
        safety_monitor.monitor_loop()
    )

if __name__ == "__main__":
    asyncio.run(main())