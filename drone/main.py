"""
Main entry point for a *single* Drone-MOB client.

This process runs *on the drone* (or in a simulator) and connects
to the Coordinator via MQTT.

Usage:
    python main.py --id scout_1
    python main.py --id payload_1
    python main.py --id utility_1
"""

import yaml
import sys
import asyncio 
import traceback
import argparse
from pathlib import Path

# --- Robust Import Logic ---
# Add the current directory to path to ensure 'core' can be imported
FILE = Path(__file__).resolve()
CORE_DIR = FILE.parent
if str(CORE_DIR) not in sys.path:
    sys.path.append(str(CORE_DIR))
# --- End Import Logic ---

# Import all core components
from core.drone import Drone, SimulatedFlightController, MavlinkController
from core.cameras.dual_camera import DualCameraSystem
from core.cameras.thermal.simulated import SimulatedThermalCamera
from core.cameras.visual.simulated import SimulatedVisualCamera
from core.logger import MissionLogger
from core.mission import MissionController
from core.config_models import Settings, DroneConfig
from core.safety import CollisionAvoider, StubObstacleSensor
from core.comms import MqttClient

# Import all strategies to pass to the controller
from strategies.search.lawnmower import create_lawnmower_search_strategy
from strategies.search.random import create_random_search_strategy
from strategies.search.vertical_ascent import create_vertical_ascent_search_strategy
from strategies.flight.direct import create_direct_flight_strategy
from strategies.flight.orbit import create_orbit_flight_strategy
from strategies.flight.precision_hover import create_precision_hover_flight_strategy


def load_config(config_path: str = "config/mission_config.yaml") -> Settings:
    """Load and validate configuration."""
    config_file = Path(__file__).parent / config_path
    try:
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
        settings = Settings(**config_data)
        return settings
    except FileNotFoundError:
        print(f"FATAL: Configuration file not found at {config_file}")
        sys.exit(1)
    except Exception as e:
        print(f"FATAL: Error validating configuration file {config_file}:\n{e}")
        sys.exit(1)

def create_cameras(config: Settings, drone_cfg: DroneConfig) -> DualCameraSystem | None:
    """Create camera system based on validated configuration."""
    
    # Drones without cameras (payload) should not create them
    if drone_cfg.role == "payload":
        print("[main] Role is 'payload', skipping camera creation.")
        return None

    thermal_cfg = config.cameras.thermal
    if thermal_cfg.type == 'simulated':
        thermal_cam = SimulatedThermalCamera(
            resolution=thermal_cfg.resolution,
            water_temp=thermal_cfg.water_temp,
            ambient_temp=thermal_cfg.ambient_temp
        )
    else:
        # TODO: Add RealThermalCamera(thermal_cfg)
        raise ValueError(f"Real thermal camera type '{thermal_cfg.type}' not implemented.")
    
    visual_cfg = config.cameras.visual
    if visual_cfg.type == 'simulated':
        visual_cam = SimulatedVisualCamera(resolution=visual_cfg.resolution)
    else:
        # TODO: Add RealVisualCamera(visual_cfg)
        raise ValueError(f"Real visual camera type '{visual_cfg.type}' not implemented.")
    
    recording_cfg = config.cameras.recording
    dual_camera = DualCameraSystem(
        thermal_cam, 
        visual_cam, 
        recording_cfg.enabled
    )
    return dual_camera

async def main():
    """Main asynchronous entry point for the Drone Client."""
    
    # --- NEW: Parse command-line arguments ---
    parser = argparse.ArgumentParser(description="Drone-MOB Client")
    parser.add_argument(
        '--id', 
        type=str, 
        required=True, 
        help="The unique ID of this drone (e.g., 'scout_1')"
    )
    args = parser.parse_args()
    drone_id = args.id
    # -----------------------------------------

    mqtt_client = None # Define here for finally block
    try:
        # 1. Load configuration
        config = load_config()
        
        # --- FIX: Find this drone's specific config ---
        drone_cfg: DroneConfig | None = None
        for d in config.drones:
            if d.id == drone_id:
                drone_cfg = d
                break
        
        if not drone_cfg:
            print(f"FATAL: No configuration found for drone with ID '{drone_id}' in mission_config.yaml")
            sys.exit(1)
        
        print(f"[main] Starting drone '{drone_id}' with role '{drone_cfg.role}'")
        # -----------------------------------------------

        # 2. Create MQTT Comms Client
        mqtt_client = MqttClient(config.mqtt, client_id=drone_id)
        await mqtt_client.connect()
        if not mqtt_client.is_connected:
            raise ConnectionError("Failed to connect to MQTT broker.")

        # 3. Create components
        # --- FIX: Select controller based on config ---
        if drone_cfg.type == 'simulated':
            base_controller = SimulatedFlightController()
            print("[main] Using SIMULATED Flight Controller")
        elif drone_cfg.type == 'real':
            # This would connect to a real drone or a SITL instance
            # TODO: Get connection string from config
            base_controller = MavlinkController(connection_string="udp:127.0.0.1:14550")
            print("[main] Using REAL (Mavlink) Flight Controller")
        else:
            raise ValueError(f"Unknown drone type in config: '{drone_cfg.type}'")
            
        obstacle_sensor = StubObstacleSensor()
        # --- FIX: Pass config to CollisionAvoider ---
        safe_controller = CollisionAvoider(
            base_controller, 
            obstacle_sensor, 
            config.safety
        )
        # --------------------------------------------
        
        drone = Drone(safe_controller, drone_id=drone_id)
        # -----------------------------------------------
        
        # Create cameras (or None if payload drone)
        dual_camera = create_cameras(config, drone_cfg) 
        
        # --- FIX: Create strategy dictionaries with correct configs ---
        search_strategies = {
            "random": create_random_search_strategy(config.vertical_ascent), # Note: RandomSearchStrategy doesn't use config, but factory expects it
            "vertical_ascent": create_vertical_ascent_search_strategy(config.vertical_ascent),
            "lawnmower": create_lawnmower_search_strategy(config.lawnmower)
        }
        flight_strategies = {
            "direct": create_direct_flight_strategy(config.precision_hover), # Note: DirectFlightStrategy doesn't use config, but factory expects it
            "precision_hover": create_precision_hover_flight_strategy(config.precision_hover),
            "orbit": create_orbit_flight_strategy(config.orbit)
        }
        # -----------------------------------------------------------

        # Create logger with drone-specific log file
        log_dir = f"{config.logging.log_dir}/{drone_id}"
        logger = MissionLogger(log_dir=log_dir, log_to_console=True) # <-- FIX: log_to_console is not a valid arg
        
        # 4. Create mission controller
        mission = MissionController(
            drone=drone,
            dual_camera=dual_camera,
            search_strategies=search_strategies, # Pass dict
            flight_strategies=flight_strategies, # Pass dict
            config=config,
            logger=logger,
            mqtt_client=mqtt_client # Pass MQTT client
        )
        
        # 5. Execute mission
        print(f"--- [Drone Client: {drone_id}] ---")
        print("Connecting to Coordinator... (Press Ctrl+C to abort)")
        print("---")
        
        # This now just listens for commands
        await mission.run()
        
    except (KeyboardInterrupt, asyncio.CancelledError):
        print(f"\n[main {drone_id}] Shutting down...")
    except Exception as e:
        print(f"[main {drone_id}] Fatal error: {e}")
        traceback.print_exc()
        sys.exit(1)
    finally:
        if mqtt_client and mqtt_client.is_connected:
            await mqtt_client.disconnect()
        print(f"[main {drone_id}] Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main())

