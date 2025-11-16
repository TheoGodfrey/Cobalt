# run.py
#
# [UPDATED]
# This is now the main executable, living in the project root.
# It correctly imports from the 'track_1' package.

import argparse
import sys

# Import Track 1 components from the 'track_1' package
from track_1.mob_rescue import MOBRescue, SearchArea
from track_1.ship_config import ShipConfig
from track_1.remote_control import RemoteControlWrapper
from track_1.hal.dji_mavic3 import DJIMavic3
from track_1.sensors.thermal_camera import ThermalCamera
from track_1.safety.safety_monitor import SafetyMonitor 

def main():
    """
    Parses command-line arguments and launches the MOB rescue mission.
    """
    
    # 1. Setup Argument Parser
    parser = argparse.ArgumentParser(
        description="COBALT Track 1: MOB Rescue Mission Launcher",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--drone_id",
        type=str,
        required=True,
        help="Connection string for the drone.\n(e.g., '192.168.1.100' for a network drone, '0' for a USB drone)"
    )
    parser.add_argument(
        "--role",
        type=str,
        required=True,
        choices=["deliver", "strobe"],
        help="The role for this drone ('deliver' or 'strobe')"
    )
    parser.add_argument(
        "--ship",
        type=str,
        default="medium",
        choices=["small", "medium", "large"],
        help="The size of the ship this drone is launching from (default: medium)."
    )
    
    args = parser.parse_args()

    # 2. Initialize hardware based on args
    print(f"[Launcher] Initializing drone: {args.drone_id}")
    try:
        # The drone_id IS the connection string
        drone = DJIMavic3(connection_string=args.drone_id)
        
        # The ThermalCamera class will find its own stream
        camera = ThermalCamera(drone)
        
        if camera.cap is None:
            raise Exception(f"Failed to open video stream for camera.")
            
    except Exception as e:
        print(f"❌ CRITICAL: Failed to connect to hardware: {e}")
        print("Please check connection and --drone_id argument.")
        sys.exit(1) # Exit the script
        
    # 3. Load ship configuration
    ship = ShipConfig(args.ship)

    # 4. Create mission instance (PASSING THE ROLE)
    mission = MOBRescue(
        drone=drone, 
        camera=camera, 
        ship_config=ship, 
        role=args.role
    )

    # 5. Add remote control capability
    # Use different ports for each role
    rc_port = 8080 if args.role == "deliver" else 8081
    print(f"[Launcher] Remote Control interface on port {rc_port}")
    mission_with_remote = RemoteControlWrapper(
        mission=mission,
        port=rc_port
    )

    # 6. Create and start Safety Monitor
    safety_monitor = SafetyMonitor(drone=drone, mission=mission)
    safety_monitor.start()

    # 7. Define search area
    # This will be based on the drone's *stubbed* starting position
    # In a real app, this would come from the ship's GCS.
    start_lat, start_lon = drone.get_position()
    radius_deg = ship.search_radius / 111111.0 # Rough conversion
    search_area = SearchArea(
        lat_bounds=(start_lat - radius_deg, start_lat + radius_deg),
        lon_bounds=(start_lon - radius_deg, start_lon + radius_deg)
    )
    
    # 8. Execute
    print(f"\n{'='*50}")
    print(f"LAUNCHING MISSION")
    print(f"   Drone ID: {args.drone_id}")
    print(f"   Ship:     {args.ship}")
    print(f"   Role:     {args.role.upper()}")
    print(f"{'='*50}\n")
    
    # This is the main blocking call that runs the whole mission
    result = mission_with_remote.execute(search_area)

    # 9. Clean up the Safety Monitor
    safety_monitor.stop()

    # 10. Print final result
    print(f"\n{'='*50}")
    print(f"MISSION COMPLETE (Drone: {args.drone_id})")
    print(f"   Final Status: {result['status']}")
    if result['status'] == 'success':
        print(f"   Target Location: {result['target']}")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()