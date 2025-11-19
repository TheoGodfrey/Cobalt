#!/bin/bash

# 1. Kill any previous instances (Cleanup)
echo "[Sim] Cleaning up old processes..."
pkill -f "python drone/main.py"
pkill -f "python hub/main.py"

# 2. Ensure MQTT Broker is running
if ! pgrep -x "mosquitto" > /dev/null; then
    echo "[Sim] Starting MQTT Broker..."
    # Attempt to start in background if installed
    mosquitto -d 2>/dev/null || echo "WARNING: Could not start mosquitto. Ensure it is running."
fi

# 3. Launch The System
# We only need to launch the Hub. It will read the config and spawn the drones.
echo "[Sim] Launching Operation Cobalt..."
echo "------------------------------------------------"

python hub/main.py --mission missions/MOB_mission.yaml --mode sim

# (The script will hang here because hub/main.py runs a loop. 
#  When you Ctrl+C, the Hub intercepts it and kills the drones.)