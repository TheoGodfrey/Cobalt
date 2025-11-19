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
echo "[Sim] Launching Operation Cobalt (Unbuffered)..."
echo "------------------------------------------------"

# Added -u to force unbuffered output so you see logs immediately
python -u hub/main.py --mission missions/MOB_mission.yaml --mode sim