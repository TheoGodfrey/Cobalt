# track1_shipping/remote_control.py

import time
import threading
from .mob_rescue import MOBRescue # Use relative import for type hint

class RemoteControlInterface:
    """[STUB] Represents the operator's RC joystick/GCS."""
    def __init__(self, port=8080):
        print(f"[STUB] Remote control interface listening on port {port}.")

    def operator_requests_control(self):
        """[STUB] Check if operator is trying to take over."""
        # Always return False for this simple demo
        return False

    def operator_releases_control(self):
        """[STUB] Check if operator has given control back."""
        return True

    def get_command(self):
        """[STUB] Get manual flight commands."""
        return "HOVER"


class RemoteControlWrapper:
    """
    Wrap any mission with remote control capability.
    Operator can take control at any time.
    """
    def __init__(self, mission: MOBRescue, control_interface: RemoteControlInterface):
        self.mission = mission
        self.control = control_interface
        self.operator_active = False

    def execute(self, *args, **kwargs):
        """Execute mission with operator override capability."""
        
        # Start mission in background thread
        mission_thread = threading.Thread(
            target=self.mission.execute,
            args=args,
            kwargs=kwargs
        )
        mission_thread.start()
        print("[RC] Mission thread started.")

        # Monitor for operator override in main thread
        while mission_thread.is_alive():
            if self.control.operator_requests_control():
                self._pause_mission()
                self._operator_control()
                self._resume_mission()

            time.sleep(0.1)
        
        print("[RC] Mission thread finished.")
        return self.mission.mission_result

    def _pause_mission(self):
        """Pause autonomous mission."""
        print("⏸️  Operator taking control! Pausing mission.")
        self.operator_active = True
        self.mission.mission_result = {"status": "paused_by_operator"}
        self.mission.drone.goto(self.mission.drone.get_position()) # Tell drone to hover

    def _operator_control(self):
        """Hand control to operator."""
        print("[RC] Operator has control.")
        while not self.control.operator_releases_control():
            cmd = self.control.get_command()
            # self.mission.drone.execute_manual_command(cmd)
            time.sleep(0.05)
        print("[RC] Operator released control.")

    def _resume_mission(self):
        """Resume autonomous mission."""
        print("▶️  Resuming autonomous mission (NOT IMPLEMENTED IN THIS STUB).")
        self.operator_active = False
        self.mission.mission_result = {"status": "aborted_by_operator"}