"""
Main entry point for the Tier 2 Comms & Charging Hub.

This process runs at the "docking station"[cite: 30]. It is a
"non-thinking component"  that enables the P2P swarm.

It runs:
1. The GcsServer (for Level 2 Local Operation) [cite: 59]
2. The SatelliteRelay (for Tier 3 Uplink) 
"""
import asyncio
import yaml
import sys
import traceback
from pathlib import Path

# --- Robust Import Logic ---
FILE = Path(__file__).resolve()
ROOT = FILE.parent.parent
CORE_PATH = ROOT / "v_0_2" / "scout_drone"
if str(CORE_PATH) not in sys.path:
    sys.path.append(str(CORE_PATH))
# --- End Import Logic ---

from drone.core.config_models import Settings
from drone.core.comms import MqttClient
from hub.gcs_server import GcsServer
from satellite_relay import SatelliteRelay

def load_config(config_path: str = "v_0_2/scout_drone/config/mission_config.yaml") -> Settings:
    """Load and validate configuration."""
    config_file = ROOT / config_path
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

async def main():
    """Main asynchronous entry point for the Tier 2 Hub."""
    mqtt_client = None
    try:
        # 1. Load configuration
        config = load_config()

        # 2. Create MQTT Comms Client for the Hub
        # This client represents the Hub's high-gain antenna [cite: 34]
        mqtt_client = MqttClient(config.mqtt, client_id="tier_2_hub")
        await mqtt_client.connect()
        if not mqtt_client.is_connected:
            raise ConnectionError("Failed to connect to MQTT broker.")

        # 3. Create GCS Server (for Level 2/3) [cite: 59, 62]
        # We pass the MQTT client to it so it can PUBLISH events
        gcs_server = GcsServer(config.gcs, mqtt_client) 

        # 4. Create the Satellite Relay 
        relay = SatelliteRelay(mqtt_client)

        # 5. Run all services concurrently
        print("[HubMain] Running all Tier 2 services (GCS, SatRelay)...")
        await asyncio.gather(
            gcs_server.run(),
            relay.run()
        )

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[HubMain] Shutting down...")
    except Exception as e:
        print(f"[HubMain] Fatal error: {e}")
        traceback.print_exc()
    finally:
        if mqtt_client and mqtt_client.is_connected:
            await mqtt_client.disconnect()
        print("[HubMain] Shutdown complete.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[HubMain] Shutdown complete.")