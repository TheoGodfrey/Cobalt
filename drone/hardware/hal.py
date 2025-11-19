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
    Includes gravity and drag approximation.
    """
    def __init__(self):
        # Start at 0,0, -10m (altitude 10m)
        self.state = DroneState(
            position_local=np.array([0.0, 0.0, -10.0]),
            velocity=np.array([0.0, 0.0, 0.0]),
            battery=100.0,
            armed=True,
            mode="GUIDED",
            heading=0.0
        )
        self.last_update = time.time()
        self.target_velocity = np.zeros(3)

    def connect(self):
        print("[MockHAL] Connected to virtual drone (3D Physics).")

    async def connect(self):
        # Async wrapper for compatibility
        self.connect()
        return True

    def get_state(self):
        return self.state

    def send_velocity_command(self, vel_cmd):
        """
        Simulates velocity control response.
        """
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        # Simple First-Order Lag (Simulate inertia/controller response)
        # tau = 0.5 seconds
        tau = 0.5
        alpha = dt / (tau + dt)
        
        self.state.velocity = (1 - alpha) * self.state.velocity + alpha * vel_cmd
        
        # Integrate Position
        self.state.position_local += self.state.velocity * dt
        
        # Simulate battery drain (propulsion cost)
        speed = np.linalg.norm(self.state.velocity)
        power = 100.0 + (speed ** 2) # Base load + drag
        drain = (power / 3600.0) * dt * 0.1 # Arbitrary scale
        self.state.battery = max(0, self.state.battery - drain)

class MavlinkHAL:
    """
    Hardware Abstraction Layer for MAVLink drones (ArduPilot/PX4).
    Implements velocity control via SET_POSITION_TARGET_LOCAL_NED.
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
        if not DRONEKIT_AVAILABLE:
            return
            
        print(f"[MavlinkHAL] Connecting to {self.conn_str}...")
        try:
            # connect() is blocking in dronekit, wrap it if needed, 
            # but usually fine for init phase.
            self.vehicle = connect(self.conn_str, wait_ready=True, timeout=60)
            print("[MavlinkHAL] Connected to Flight Controller.")
            
            # Set up listeners
            self.vehicle.add_attribute_listener('location.local_frame', self._on_location)
            self.vehicle.add_attribute_listener('velocity', self._on_velocity)
            self.vehicle.add_attribute_listener('battery', self._on_battery)
            
        except Exception as e:
            print(f"[MavlinkHAL] Connection Failed: {e}")
            raise

    def _on_location(self, vehicle, name, msg):
        # Update NED Position
        if msg:
            # ArduPilot local frame: North, East, Down
            self.state.position_local = np.array([msg.north, msg.east, msg.down])

    def _on_velocity(self, vehicle, name, msg):
        # Update NED Velocity
        if msg:
            self.state.velocity = np.array([msg[0], msg[1], msg[2]])

    def _on_battery(self, vehicle, name, msg):
        if msg:
            self.state.battery = msg.level

    def get_state(self):
        if self.vehicle:
            self.state.armed = self.vehicle.armed
            self.state.mode = self.vehicle.mode.name
            self.state.heading = math.radians(self.vehicle.heading)
        return self.state

    def send_velocity_command(self, vel_cmd):
        """
        Sends a velocity vector [vx, vy, vz] to the FC using MAVLink.
        Frame: MAV_FRAME_LOCAL_NED
        """
        if not self.vehicle or not DRONEKIT_AVAILABLE:
            return

        # Create MAVLink message
        # mask: 0b0000111111000111
        # bit 0,1,2 = Position (Ignored) -> 1
        # bit 3,4,5 = Velocity (Used)    -> 0
        # bit 6,7,8 = Accel (Ignored)    -> 1
        # bit 9     = Force (Ignored)    -> 0 (Not supported usually)
        # bit 10    = Yaw (Ignored)      -> 1
        # bit 11    = Yaw Rate (Ignored) -> 1 (Let FC handle yaw to face velocity?)
        
        # We want the drone to face the direction of travel naturally
        # ArduPilot 'GUIDED' mode handles yaw turn coordination if we don't specify yaw.
        type_mask = 0b0000111111000111 

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            type_mask, # type_mask
            0, 0, 0, # x, y, z positions (not used)
            vel_cmd[0], vel_cmd[1], vel_cmd[2], # vx, vy, vz velocity in m/s
            0, 0, 0, # x, y, z acceleration (not used)
            0, 0     # yaw, yaw_rate (not used)
        )

        self.vehicle.send_mavlink(msg)
        # Note: Need to send this at > 2Hz or drone stops