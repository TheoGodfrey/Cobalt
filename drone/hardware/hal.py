import time
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional

# For real hardware, we need dronekit or pymavlink
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    mavutil = None 
    class VehicleMode:
        def __init__(self, name): self.name = name

@dataclass
class DroneState:
    """Standardized state object for the HAL to return."""
    position_local: np.ndarray # [x, y, z] (NED, meters)
    velocity: np.ndarray       # [vx, vy, vz] (NED, m/s)
    battery: float             # %
    armed: bool
    mode: str
    heading: float             # radians

class MockHAL:
    """
    Simulates a drone with 3D physics for testing logic without hardware.
    """
    def __init__(self):
        self.state = DroneState(
            position_local=np.array([0.0, 0.0, -10.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            battery=100.0,
            armed=True,
            mode="GUIDED",
            heading=0.0
        )
        self.last_update = time.time()
        self.home_location = np.array([0.0, 0.0, -5.0]) # Target for RTL

    def connect(self):
        print("[MockHAL] Connected to virtual drone (3D Physics).")

    async def connect(self):
        self.connect()
        return True

    def get_state(self):
        return self.state

    def land(self):
        """Simulates an emergency landing command."""
        if self.state.mode != "LAND":
            print("[MockHAL] *** EMERGENCY LANDING INITIATED ***")
            self.state.mode = "LAND"
            self.state.velocity = np.array([0.0, 0.0, 1.5]) # Descend

    def return_to_launch(self):
        """Simulates a Return to Launch command."""
        if self.state.mode != "RTL":
            print("[MockHAL] *** RETURNING TO LAUNCH (RTL) ***")
            self.state.mode = "RTL"
            # Velocity will be handled in physics loop

    def send_velocity_command(self, vel_cmd):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        # --- MODE OVERRIDES ---
        if self.state.mode == "LAND":
            # Ignore input, maintain descent
            self.state.velocity = np.array([0.0, 0.0, 1.5])
            
        elif self.state.mode == "RTL":
            # Ignore input, fly to home
            vec_to_home = self.home_location - self.state.position_local
            dist = np.linalg.norm(vec_to_home)
            
            if dist < 1.0:
                print("[MockHAL] Home Reached. Landing...")
                self.land()
            else:
                # Fly towards home at 10m/s
                direction = vec_to_home / dist
                self.state.velocity = direction * 10.0
        else:
            # GUIDED mode: Accept input
            vel_cmd = np.array(vel_cmd)
            tau = 0.5
            alpha = dt / (tau + dt)
            self.state.velocity = (1 - alpha) * self.state.velocity + alpha * vel_cmd

        # Integrate Position
        self.state.position_local = self.state.position_local + (self.state.velocity * dt)
        
        # Simulate battery drain
        speed = np.linalg.norm(self.state.velocity)
        power = 100.0 + (speed ** 2) 
        drain = (power / 3600.0) * dt * 0.1 
        self.state.battery = max(0, self.state.battery - drain)

class MavlinkHAL:
    """
    Hardware Abstraction Layer for MAVLink drones (ArduPilot/PX4).
    """
    def __init__(self, connection_string):
        self.conn_str = connection_string
        self.vehicle = None
        self.state = DroneState(
            position_local=np.zeros(3),
            velocity=np.zeros(3),
            battery=100.0,
            armed=False,
            mode="UNKNOWN",
            heading=0.0
        )
        
        if not DRONEKIT_AVAILABLE:
            print("[MavlinkHAL] WARNING: DroneKit not installed. Falling back to Mock behavior.")

    async def connect(self):
        if not DRONEKIT_AVAILABLE: return
        print(f"[MavlinkHAL] Connecting to {self.conn_str}...")
        try:
            self.vehicle = connect(self.conn_str, wait_ready=True, timeout=60)
            print("[MavlinkHAL] Connected.")
            self.vehicle.add_attribute_listener('location.local_frame', self._on_location)
            self.vehicle.add_attribute_listener('velocity', self._on_velocity)
            self.vehicle.add_attribute_listener('battery', self._on_battery)
        except Exception as e:
            print(f"[MavlinkHAL] Connection Failed: {e}")
            raise

    def _on_location(self, vehicle, name, msg):
        if msg: self.state.position_local = np.array([msg.north, msg.east, msg.down])

    def _on_velocity(self, vehicle, name, msg):
        if msg: self.state.velocity = np.array([msg[0], msg[1], msg[2]])

    def _on_battery(self, vehicle, name, msg):
        if msg: self.state.battery = msg.level

    def get_state(self):
        if self.vehicle:
            self.state.armed = self.vehicle.armed
            self.state.mode = self.vehicle.mode.name
            self.state.heading = math.radians(self.vehicle.heading)
        return self.state

    def land(self):
        if not self.vehicle: return
        print(f"[MavlinkHAL] COMMAND: LAND")
        self.vehicle.mode = VehicleMode("LAND")

    def return_to_launch(self):
        if not self.vehicle: return
        print(f"[MavlinkHAL] COMMAND: RTL")
        self.vehicle.mode = VehicleMode("RTL")

    def send_velocity_command(self, vel_cmd):
        if not self.vehicle or not DRONEKIT_AVAILABLE or mavutil is None: return
        # Safety: Only send velocity in GUIDED mode
        if self.vehicle.mode.name not in ["GUIDED"]: return

        type_mask = 0b0000111111000111 
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask, 0, 0, 0,
            vel_cmd[0], vel_cmd[1], vel_cmd[2], 0, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)