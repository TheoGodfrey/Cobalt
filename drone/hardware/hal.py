import time
import numpy as np
from dataclasses import dataclass

@dataclass
class DroneState:
    """Standardized state object for the HAL to return."""
    position_local: np.ndarray
    velocity: np.ndarray
    battery: float
    armed: bool

class MockHAL:
    """
    Simulates a drone for testing logic without hardware.
    """
    def __init__(self):
        # Start at 0,0, -10m (altitude)
        self.state = DroneState(
            position_local=np.array([0.0, 0.0, -10.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            battery=100.0,
            armed=True
        )
        self.last_update = time.time()

    def connect(self):
        print("[MockHAL] Connected to virtual drone.")

    def get_state(self):
        return self.state

    def send_velocity_command(self, vel_cmd):
        """
        Simple physics integration for the mock.
        Updates position based on the commanded velocity.
        """
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        # Update mock state
        self.state.velocity = np.array(vel_cmd)
        self.state.position_local += self.state.velocity * dt

class MavlinkHAL:
    """
    Hardware Abstraction Layer for MAVLink drones (e.g., ArduPilot).
    """
    def __init__(self, connection_string):
        self.conn_str = connection_string
        self.state = DroneState(
            position_local=np.array([0.0, 0.0, 0.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            battery=100.0,
            armed=False
        )
        self.vehicle = None

    def connect(self):
        print(f"[MavlinkHAL] Connecting to {self.conn_str}...")
        # NOTE: In a real deployment, import dronekit here
        # from dronekit import connect
        # self.vehicle = connect(self.conn_str, wait_ready=True)
        print("[MavlinkHAL] Connected (Placeholder).")

    def get_state(self):
        # In real implementation:
        # pos = self.vehicle.location.local_frame
        # self.state.position_local = np.array([pos.north, pos.east, pos.down])
        return self.state

    def send_velocity_command(self, vel):
        # In real implementation, send SET_POSITION_TARGET_LOCAL_NED
        pass