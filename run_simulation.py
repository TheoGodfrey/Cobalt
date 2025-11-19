import sys
import os
import subprocess
import time
import shutil

def main():
    print("[Launcher] Initializing Cobalt Environment...")

    # 1. Set PYTHONPATH to current directory so 'core' modules are found
    current_dir = os.getcwd()
    os.environ["PYTHONPATH"] = current_dir + os.pathsep + os.environ.get("PYTHONPATH", "")
    
    # 2. Check Dependencies
    required_libs = ["yaml", "numpy", "cv2", "paho.mqtt.client", "scipy"]
    missing = []
    for lib in required_libs:
        try:
            # Handle mapping from pip name to import name
            import_name = lib
            if lib == "cv2": import_name = "cv2"
            if lib == "paho.mqtt.client": import_name = "paho.mqtt"
            
            __import__(import_name)
        except ImportError:
            missing.append(lib)
    
    if missing:
        print(f"\n[Launcher] CRITICAL ERROR: Missing dependencies: {', '.join(missing)}")
        print(f"[Launcher] Please run: pip install pyyaml numpy opencv-python paho-mqtt scipy\n")
        return

    # 3. Cleanup Old Processes (Best Effort)
    # On Windows, pkill doesn't exist, so we skip or use taskkill if needed.
    # This prevents the 'Address already in use' errors.
    
    # 4. Start MQTT Broker (Mosquitto) check
    # On Windows, checking for mosquitto via command line is tricky.
    # We assume the user has it running or the code handles offline mode.
    
    # 5. Launch Hub
    print("[Launcher] Starting Hub (hub/main.py)...")
    print("------------------------------------------")
    
    cmd = [sys.executable, "-u", "hub/main.py", "--mission", "missions/MOB_mission.yaml", "--mode", "sim"]
    
    try:
        # We use shell=False to ensure signals are handled correctly
        result = subprocess.run(cmd, env=os.environ)
    except KeyboardInterrupt:
        print("\n[Launcher] Simulation stopped by user.")

if __name__ == "__main__":
    main()