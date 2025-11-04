"""
Hub Main Entry Point
This file initializes and runs all Tier 2 Hub services.
"""

import asyncio
import os
from drone.core.cross_cutting.communication import MqttClient
# --- FIX 3.2: Import config loader ---
from drone.core.utils.config_models import load_config
# --- End of FIX 3.2 ---
from .fleet_coordinator import FleetCoordinator
from .gcs_server import GcsServer
from .charging_monitor import ChargingMonitor
from .satellite_relay import SatelliteRelay

# --- Configuration ---
# --- FIX 3.2: Load config from file ---
CONFIG_FILE = os.environ.get("COBALT_CONFIG", "config/system_config.yaml")
config = load_config(CONFIG_FILE)
network_config = config.get("network", {})
fleet_config = config.get("fleet", {})

MQTT_HOST = os.environ.get("MQTT_HOST", network_config.get("mqtt_broker_host", "localhost"))
MQTT_PORT = int(os.environ.get("MQTT_PORT", network_config.get("mqtt_broker_port", 1883)))
GCS_PORT = int(os.environ.get("GCS_PORT", network_config.get("gcs_server_port", 8765)))
# --- End of FIX 3.2 ---

async def main():
    """
    Initializes and runs all concurrent hub tasks.
    """
    
    # 1. Initialize Communication Layer
    # The Hub needs its own MQTT client to act as the master
    hub_comms = MqttClient(client_id="hub_master", host=MQTT_HOST, port=MQTT_PORT)
    await hub_comms.connect()

    # 2. Initialize Core Hub Components
    # --- FIX 3.2: Pass fleet_config to FleetCoordinator ---
    fleet_coord = FleetCoordinator(hub_comms, fleet_config)
    # --- End of FIX 3.2 ---
    gcs_server = GcsServer(fleet_coord, port=GCS_PORT)
    charging_mon = ChargingMonitor(num_pads=4)
    sat_relay = SatelliteRelay()
    
    # 3. Link components if needed
    # (e.g., FleetCoordinator needs to know about charging status)
    charging_mon.add_listener(
        lambda pad_id, status: 
            print(f"Callback: Pad {pad_id} is now {status.name}")
            # fleet_coord.update_pad_status(pad_id, status) # <-- Example
    )

    print("[HubMain] All components initialized. Starting concurrent tasks...")

    # 4. Run all components concurrently
    await asyncio.gather(
        hub_comms.run(),          # MQTT client loop
        fleet_coord.listen(),     # FleetCoordinator's subscription loop
        gcs_server.run(),         # GCS WebSocket server
        charging_mon.run(),       # Charging pad monitor loop
        sat_relay.run()           # Satellite uplink loop
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[HubMain] Shutting down.")
