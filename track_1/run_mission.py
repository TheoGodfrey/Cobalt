# run_mission.py
#
# [UPDATED]
# This is now a command-line executable for the Track 1 mission.
# It parses drone_id and role from the command line and starts
# the mission immediately.

import argparse
import sys

# Import Track 1 components
from track1_shipping.mob_rescue import MOBRescue, SearchArea
from track1_shipping.ship_config import ShipConfig
from track1_shipping.remote_control import RemoteControlWrapper, RemoteControlInterface

# Import Shared components
from shared.hal.dji_mavic3 import DJIMavic3
from shared.sensors.thermal_camera import ThermalCamera

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
        
        # For USB-based cameras, the ID might be the same as the drone
        # For network cameras, the ThermalCamera class will have the stream URL
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

    # 5. Add remote control capability (stub)
    # Use different ports for each role to avoid conflicts
    # if running on the same machine.
    rc_port = 8080 if args.role == "deliver" else 8081
    print(f"[Launcher] Remote Control interface on port {rc_port}")
    control_interface = RemoteControlInterface(port=rc_port)
    mission_with_remote = RemoteControlWrapper(mission, control_interface)

    # 6. Define search area
    # This will be based on the drone's *stubbed* starting position
    # In a real app, this would come from the ship's GCS.
    start_lat, start_lon = drone.get_position()
    radius_deg = ship.search_radius / 111000 # Rough conversion
    search_area = SearchArea(
        lat_bounds=(start_lat - radius_deg, start_lat + radius_deg),
        lon_bounds=(start_lon - radius_deg, start_lon + radius_deg)
    )

    # 7. Execute
    print(f"\n{'='*50}")
    print(f"LAUNCHING MISSION")
    print(f"   Drone ID: {args.drone_id}")
    print(f"   Ship:     {args.ship}")
    print(f"   Role:     {args.role.upper()}")
    print(f"{'='*50}\n")
    
    # This is the main blocking call that runs the whole mission
    result = mission_with_remote.execute(search_area)

    # 8. Print final result
    print(f"\n{'='*50}")
    print(f"MISSION COMPLETE (Drone: {args.drone_id})")
    print(f"   Final Status: {result['status']}")
    if result['status'] == 'success':
        print(f"   Target Location: {result['target']}")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()