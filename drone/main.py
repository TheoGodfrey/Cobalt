<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
import asyncio
import argparse
import numpy as np
import yaml
import math
import time
import signal # <-- ADDED for Bug 4
import platform # <-- ADDED for Bug 4

# Core Infrastructure
from core.bus import MessageBus
from core.supervisor import ServiceSupervisor
from core.comms import Comms
from core.types import DroneState, TelemetryMessage

# Logic & State
from core.state.world_state import WorldState
from core.mission.manager import MissionManager
from core.maths.solver import ProbabilisticSolver

# Hardware & Safety
from drone.hardware.hal import MockHAL, MavlinkHAL
from drone.safety.safety_monitor import SafetyMonitor

# Vision (The parts we removed earlier)
from drone.hardware.camera import CameraSensor
from drone.hardware.lidar import LidarSensor
from core.detectors.thermal.manager import DetectorManager
from core.detectors.visual.colour import ColorBlobDetector
from core.maths.georeference import Georeferencer

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True)
    parser.add_argument("--hal", default="mock")
    parser.add_argument("--mission", default="missions/MOB_mission.yaml")
    args = parser.parse_args()

    print(f"[Main] Initializing Drone {args.id}...")

    # 1. Infrastructure (The Nervous System)
    bus = MessageBus()
    supervisor = ServiceSupervisor(bus)
    
    # 2. State & Configuration (The Brain)
    world = WorldState()
    world.my_id = args.id
    
    try:
        with open(args.mission) as f: mission_plan = yaml.safe_load(f)
        manager = MissionManager(mission_plan, world, ProbabilisticSolver(), SafetyMonitor(world))
    except Exception as e:
        print(f"[Main] Error loading mission: {e}")
        manager = None

    # 3. Hardware (The Body)
    if args.hal == "mavlink":
        hal = MavlinkHAL("udp:127.0.0.1:14550")
        camera = CameraSensor(mode="real")
        lidar = LidarSensor(mode="real")
    else:
        hal = MockHAL()
        camera = CameraSensor(mode="mock")
        lidar = LidarSensor(mode="mock")

    # Shared Objects
    solver = ProbabilisticSolver() # Control Math
    safety = SafetyMonitor(world)  # Safety Limits
    comms = Comms(args.id, bus)    # Radio

    # --- Signal Handlers (Required for Bug 4 Fix) ---
    def emergency_kill(signum, frame):
        print(f"\n[Main] Signal {signum} received. Initiating emergency shutdown.")
        if hal: hal.land() # Attempt a controlled landing
        # Stop the asyncio loop for graceful exit
        asyncio.get_running_loop().stop()
        
    def handle_rtl(signum, frame):
        print(f"\n[Main] Signal {signum} received. Returning to Launch (RTL).")
        if hal: hal.return_to_launch()

    def handle_land(signum, frame):
        print(f"\n[Main] Signal {signum} received. Landing Now (LAND).")
        if hal: hal.land()

    # --- Bug 4 Fix: Signal Registration (Replaced lines 68-79) ---
    signal.signal(signal.SIGINT, emergency_kill)
    signal.signal(signal.SIGTERM, emergency_kill)

    if platform.system() != "Windows":
        signal.signal(signal.SIGUSR1, handle_rtl)
        signal.signal(signal.SIGUSR2, handle_land)
        print("[Main] Listening for SIGUSR1 (RTL) and SIGUSR2 (LAND).")
    else:
        print("[Main] On Windows: Use MQTT commands for RTL/LAND (no SIGUSR signals)")
        
    # ---------------------------------------------------------
    # SERVICE 1: Guidance (The Pilot)
    # Priority: HIGH. Must never block.
    # ---------------------------------------------------------
    async def service_guidance():
        print("[Guidance] Service Started.")
        if manager: manager.start()
        await hal.connect()
        
        while True:
            t0 = asyncio.get_running_loop().time()
            
            # A. Get Strict State
            state: DroneState = hal.get_state()

            # --- BUG FIX 5: Manual Override Pause/Resume ---
            if manager:
                # 1. Detect MANUAL override and pause mission state
                if state.mode == "MANUAL" and not manager.paused:
                    manager.pause_mission("manual_override")
                # 2. Detect GUIDED resume after manual override
                elif state.mode == "GUIDED" and manager.paused:
                    manager.resume_mission()
            # -----------------------------------------------
            
            # B. Update Mission Triggers (Did we find it?)
            # Skip mission checks if we are manually flown or paused
            if manager and not manager.paused: 
                manager.check_triggers(state)
            
            # C. Solve for Velocity
            if manager and not manager.paused:
                # Mission is active: calculate command from solver
                raw_cmd = solver.solve(state)
            else:
                # Mission is paused (MANUAL, or awaiting resume): command zero velocity
                raw_cmd = np.zeros(3)
                
            # D. Safety Filter (Dynamic Home & Geofence)
            safe_cmd = safety.filter_velocity(raw_cmd, state)
            
            # E. Actuate
            hal.send_velocity_command(safe_cmd)
            
            # F. Telemetry (Fire & Forget)
            telem = TelemetryMessage(
                id=args.id,
                pos=state.position_local.tolist(),
                vel=state.velocity.tolist(),
                batt=state.battery,
                hdg=state.heading,
                mode=state.mode,
                phase=manager.current_phase_idx if manager else 0
            )
            # Re-added 'posture' from the previously fixed version (Bug 3 context)
            telem_dict = telem.model_dump()
            telem_dict['posture'] = manager.current_posture if manager else "none"
            
            await bus.publish("comms/send", {"topic": "fleet/telemetry", "payload": telem_dict})
            
            # G. Heartbeat
            await bus.publish("system/heartbeat", {"service": "guidance"})
            
            # H. Loop Timing (Strict 10Hz)
            elapsed = asyncio.get_running_loop().time() - t0
            await asyncio.sleep(max(0, 0.1 - elapsed))

    # ---------------------------------------------------------
    # SERVICE 2: Vision (The Co-Pilot/Observer)
    # Priority: MEDIUM. Can run slower (e.g., 5Hz).
    # ---------------------------------------------------------
    async def service_vision():
        print("[Vision] Service Started.")
        
        # Initialize Detectors locally (Thread-safe)
        detector_mgr = DetectorManager()
        detector_mgr.add_detector(ColorBlobDetector(color_range="orange"))
        georef = Georeferencer(fov_deg_x=60.0)
        
        while True:
            t0 = time.time()
            
            # A. Get Data (Blocking I/O is okay here, won't stop guidance)
            # Note: In real async, we'd use run_in_executor for heavy CV calls
            frame = camera.get_frame()
            state = hal.get_state() # Get latest state for georeferencing
            
            # B. Process
            detections = detector_mgr.process(frame)
            
            # C. Georeference & Publish
            for d in detections:
                # Simple Pinhole Model
                cx, cy = d.bbox[0] + d.bbox[2]/2, d.bbox[1] + d.bbox[3]/2
                
                # Get Lidar Range (Simulated raycast)
                # ( Simplified: Center ray for now )
                lidar_dist = lidar.get_range_at_angle(0, 0) 
                
                world_pos = georef.pixel_to_world(cx, cy, state, lidar_range=lidar_dist)
                
                if world_pos is not None:
                    # Notify the Bus!
                    print(f"[{args.id}] SEE: {d.label} at {world_pos}")
                    
                    target_msg = {
                        'pos': world_pos.tolist(),
                        'label': d.label,
                        'conf': d.confidence,
                        'id': args.id
                    }
                    
                    # Publish locally (updates WorldState)
                    # And remote (Comms will pick this up via listener if we wired it)
                    await bus.publish("vision/detection", target_msg)
                    await bus.publish("comms/send", {"topic": "fleet/target", "payload": target_msg})

            # D. Heartbeat
            await bus.publish("system/heartbeat", {"service": "vision"})
            
            # Run at 5Hz (slower than guidance)
            await asyncio.sleep(0.2)

    # ---------------------------------------------------------
    # SERVICE 3: Comms (The Radio Operator)
    # ---------------------------------------------------------
    async def service_comms():
        await comms.run()

    # ---------------------------------------------------------
    # SERVICE 4: Listener (The Ears)
    # Handles incoming data from the Bus
    # ---------------------------------------------------------
    async def service_listener():
        # Updates Home Position if Hub moves
        async def on_home_update(data):
            safety.update_home(data['pos'])
        
        # Updates World State with detections from OTHER drones
        async def on_network_target(data):
            world.targets.update(data['pos'], data['label'], data['conf'])

        # --- Handler for fleet position synchronization (Bug 3 Fix) ---
        async def on_fleet_telemetry(data):
            drone_id = data.get('id')
            pos = data.get('pos') # List of floats
            
            # Skip my own telemetry
            if drone_id and pos and (drone_id != args.id):
                # We assume WorldState has 'fleet_positions' initialized
                world.fleet_positions[drone_id] = np.array(pos)
                print(f"[Fleet] Updated {drone_id} position: {pos}")
        # ----------------------------------------------------------
            
        bus.subscribe("fleet/home", on_home_update)
        bus.subscribe("fleet/target", on_network_target)
        # --- Subscribe to telemetry for swarm equation (Bug 3 Fix) ---
        bus.subscribe("fleet/telemetry", on_fleet_telemetry)
        # ----------------------------------------------------------
        
        while True:
            await bus.publish("system/heartbeat", {"service": "listener"})
            await asyncio.sleep(1.0)

    # 4. Register & Launch
    # This is where we prevent the "God Script" problem. 
    # If Vision crashes, Guidance lives.
    supervisor.register("comms", service_comms)
    supervisor.register("guidance", service_guidance)
    supervisor.register("vision", service_vision)
    supervisor.register("listener", service_listener)

    print("[Main] All Services Registered. Starting Supervisor.")
    await supervisor.start()
=======
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

=======
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

>>>>>>> Stashed changes
=======
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

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        print("\n[Main] Shutting down...")
=======
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        print("\n[Main] Interrupted")
    except Exception as e:
        print(f"\n[Main] Fatal error: {e}")
        import sys
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        sys.exit(1)
>>>>>>> Stashed changes
=======
        sys.exit(1)
>>>>>>> Stashed changes
=======
        sys.exit(1)
>>>>>>> Stashed changes
