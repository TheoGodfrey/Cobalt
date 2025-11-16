# track1_shipping/remote_control.py

import time
import threading
from typing import Optional, Dict, Any

from track_1.mob_rescue import MOBRescue


class SatelliteUplink:
    """
    Simulates a high-latency satellite command uplink (Starlink).
    """
    def __init__(self, port: int):
        print(f"[SATCOM] Uplink active on port {port}. Latency: 0.5s")
        self.command_queue = [] 
        self.LATENCY_SECONDS = 0.5
        # Add a lock for thread-safe command queuing
        self._lock = threading.Lock() 

    def check_for_commands(self) -> Optional[Dict[str, Any]]:
        """
        Simulates a high-latency check for new commands.
        This is a BLOCKING call.
        """
        # 1. Simulate satellite link latency
        time.sleep(self.LATENCY_SECONDS) 
        
        # 2. Check for a command
        with self._lock:
            if self.command_queue:
                command = self.command_queue.pop(0) 
                print(f"[SATCOM] 🛰️  Command received: {command}")
                return command
        return None

    def send_command(self, command: Dict[str, Any]):
        """
        A helper method for testing. Call this from another thread
        to simulate an operator sending a command.
        """
        print(f"[Operator] Sending command via SATCOM: {command}")
        with self._lock:
            self.command_queue.append(command)


class RemoteControlWrapper:
    """
    [UPDATED]
    Wraps the mission and runs a parallel "satellite listener" thread.
    This listener is now a simple state machine that can be in
    "AUTONOMOUS" or "MANUAL" mode.
    """
    def __init__(self, mission: 'MOBRescue', port: int):
        self.mission = mission
        self.satellite_link = SatelliteUplink(port=port)
        
        # --- NEW STATE MACHINE ---
        self.control_mode = "AUTONOMOUS" # AUTONOMOUS | MANUAL
        
        self._listener_should_run = True
        self._listener_thread = None
        self._mission_thread = None

    def execute(self, *args, **kwargs):
        """
        Execute mission and start satellite listener in parallel.
        """
        
        # 1. Start satellite listener in background
        self._listener_should_run = True
        self._listener_thread = threading.Thread(
            target=self._satellite_command_loop
        )
        self._listener_thread.start()
        print("[RC] Satellite listener thread started.")

        # 2. Start mission in its own background thread
        self._mission_thread = threading.Thread(
            target=self.mission.execute,
            args=args,
            kwargs=kwargs
        )
        self._mission_thread.start()
        print("[RC] Mission thread started.")

        # 3. Wait for mission to finish
        self._mission_thread.join()
        print("[RC] Mission thread finished.")

        # 4. Stop the satellite listener
        self._listener_should_run = False
        self._listener_thread.join()
        print("[RC] Satellite listener thread stopped.")
        
        return self.mission.mission_result

    def _satellite_command_loop(self):
        """
        [UPDATED] This is now the *only* loop that polls the satellite.
        It behaves differently based on self.control_mode.
        """
        while self._listener_should_run:
            # This call blocks for ~0.5 seconds
            command = self.satellite_link.check_for_commands()
            
            if not command:
                continue
            
            # --- Handle commands based on mode ---
            if self.control_mode == "AUTONOMUS":
                self._handle_autonomous_commands(command)
            elif self.control_mode == "MANUAL":
                self._handle_manual_commands(command)

    def _handle_autonomous_commands(self, command: Dict[str, Any]):
        """
        Handles commands while the mission is running itself.
        """
        if command['type'] == 'ABORT_RTH':
            print("[RC] Received ABORT_RTH command! Triggering abort.")
            self.mission.abort_mission()
        
        elif command['type'] == 'GOTO_WAYPOINT':
            # Operator can redirect the *entire* mission
            lat = command['lat']
            lon = command['lon']
            alt = command['alt']
            print(f"[RC] Received GOTO command! Aborting mission and flying to {lat, lon}@{alt}m.")
            self.mission.abort_and_redirect(
                position=(lat, lon), 
                altitude=alt
            )
        
        elif command['type'] == 'PAUSE':
            # --- This is the handoff ---
            print("[RC] Received PAUSE command! Switching to MANUAL control.")
            self.control_mode = "MANUAL"
            self.mission.pause_mission() # Tell mission to just hover
            print("[RC] Operator has manual (satellite) control.")
            
    def _handle_manual_commands(self, command: Dict[str, Any]):
        """
        Handles commands while the operator is in manual control.
        """
        # --- Command: Go to a specific GPS point ---
        if command['type'] == 'GOTO_WAYPOINT':
            lat = command['lat']
            lon = command['lon']
            alt = command['alt']
            print(f"[RC] Manual Control: Flying to {lat, lon}@{alt}m.")
            # Directly command the drone (using the HAL)
            self.mission.drone.goto(
                position=(lat, lon), 
                altitude=alt
            )
            print("[RC] Manual move complete. Awaiting next command.")

        # --- Command: Perform a specific action ---
        elif command['type'] == 'MANUAL_CMD':
            cmd = command.get('cmd')
            print(f"[RC] Got manual action: {cmd}")
            if cmd == "DROP_PAYLOAD":
                self.mission.drone.drop_payload()
            elif cmd == "STROBE_ON":
                self.mission.drone.set_strobe(True)
            elif cmd == "STROBE_OFF":
                self.mission.drone.set_strobe(False)
            
        # --- Command: Exit manual mode and resume mission ---
        elif command['type'] == 'RESUME_MISSION':
            # --- This is the handoff back ---
            print("[RC] Operator released control. Resuming autonomous logic.")
            self.control_mode = "AUTONOMOUS"
            self.mission.resume_mission() # Tell mission to un-pause
        
        # --- Command: Abort from manual mode ---
        elif command['type'] == 'ABORT_RTH':
            print("[RC] Received ABORT_RTH command! Aborting manual control.")
            self.control_mode = "AUTONOMOUS" # (Will be aborted anyway)
            self.mission.abort_mission() # This will trigger RTH