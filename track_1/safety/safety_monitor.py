# shared/safety/safety_monitor.py
import time
import threading
from typing import List

# Import HAL and Mission for type hints
from shared.hal.dji_mavic3 import DJIMavic3
from track1_shipping.mob_rescue import MOBRescue
from .rules import SafetyRule, BatteryRule, AltitudeRule

class SafetyMonitor:
    """
    Runs in a background thread to constantly check safety rules.
    Has the authority to take control from the mission.
    """
    def __init__(self, drone: DJIMavic3, mission: MOBRescue):
        self.drone = drone
        self.mission = mission
        self.rules: List[SafetyRule] = [
            BatteryRule(min_percent=25.0),
            AltitudeRule(max_altitude_m=120.0),
            # Add more rules here
        ]
        
        self.check_interval_sec = 1.0 # Check rules 1x per second
        self._is_running = False
        self._thread = None
        self._breached = False

    def start(self):
        """Start the safety monitor loop in a background thread."""
        if self._is_running:
            return
            
        print("[Safety] Starting safety monitor...")
        self._is_running = True
        self._thread = threading.Thread(target=self._monitor_loop)
        self._thread.start()

    def stop(self):
        """Stop the safety monitor loop."""
        print("[Safety] Stopping safety monitor...")
        self._is_running = False
        if self._thread:
            self._thread.join()
        print("[Safety] Monitor stopped.")

    def _monitor_loop(self):
        """The main loop for the thread."""
        while self._is_running:
            if self._breached:
                # We have breached a rule and are in a safe state.
                # Do nothing but wait for the system to shut down.
                break
                
            try:
                # 1. Get current drone state
                telemetry = self.drone.get_telemetry()
                
                # 2. Check all rules
                for rule in self.rules:
                    breach_message = rule.check(telemetry)
                    if breach_message:
                        self._handle_breach(rule, breach_message)
                        break # Stop checking, we are now in fail-safe
                
            except Exception as e:
                print(f"❌ [Safety] Error in monitor loop: {e}")
                
            time.sleep(self.check_interval_sec)

    def _handle_breach(self, rule: SafetyRule, message: str):
        """
        A rule has been breached. Take immediate, drastic action.
        """
        print(f"🚨🚨🚨 SAFETY BREACH 🚨🚨🚨")
        print(f"🚨 Rule: {message}")
        
        self._breached = True
        
        # 1. Tell the mission to abort NOW
        print("[Safety] Commanding mission to abort...")
        self.mission.abort_mission() # This sets mission_status = ABORT_RTH
        
        # 2. Take direct HAL control to enforce safety
        action = rule.get_safety_action()
        print(f"[Safety] Enforcing safety action: {action}")
        
        if action == "RTH":
            # This will take over and fly home
            self.drone.return_to_launch() 
        elif action == "HOVER":
            # This just stops the drone
            self.drone.hover()
            
        # 3. Stop this monitor, its job is done.
        self._is_running = False
        print("[Safety] Monitor has enforced action and is shutting down.")