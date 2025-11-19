import time
import numpy as np
import math
from core.types import DroneState

# For real hardware, we need dronekit or pymavlink
try:
    from dronekit import connect, VehicleMode
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    mavutil = None 
    class VehicleMode:
        def __init__(self, name): self.name = name

class MockHAL:
    """
    Simulates a drone with 3D physics for testing logic without hardware.
    """
    def __init__(self):
        self.position = np.array([0.0, 0.0, -10.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.battery = 100.0
        self.mode = "GUIDED"
        self.heading = 0.0
        self.last_update = time.time()
        self.home_location = np.array([0.0, 0.0, -5.0])

    async def connect(self):
        print("[MockHAL] Connected to virtual drone (3D Physics).")
        return True

    def get_state(self) -> DroneState:
        return DroneState(
            position_local=self.position,
            velocity=self.velocity,
            battery=self.battery,
            armed=True,
            mode=self.mode,
            heading=self.heading,
            timestamp=time.time()
        )

    def land(self):
        if self.mode != "LAND":
            print("[MockHAL] *** EMERGENCY LANDING INITIATED ***")
            self.mode = "LAND"
            self.velocity = np.array([0.0, 0.0, 1.5])

    def return_to_launch(self):
        if self.mode != "RTL":
            print("[MockHAL] *** RETURNING TO LAUNCH (RTL) ***")
            self.mode = "RTL"

    def send_velocity_command(self, vel_cmd):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        if self.mode == "LAND":
            self.velocity = np.array([0.0, 0.0, 1.5])
        elif self.mode == "RTL":
            vec_to_home = self.home_location - self.position
            dist = np.linalg.norm(vec_to_home)
            if dist < 1.0:
                self.land()
            else:
                self.velocity = (vec_to_home / dist) * 10.0
        else:
            # GUIDED
            vel_cmd = np.array(vel_cmd)
            tau = 0.5
            alpha = dt / (tau + dt)
            self.velocity = (1 - alpha) * self.velocity + alpha * vel_cmd

        self.position += self.velocity * dt
        
        # Battery Drain
        speed = np.linalg.norm(self.velocity)
        drain = (100.0 + speed**2) / 3600.0 * dt * 0.1
        self.battery = max(0, self.battery - drain)

class MavlinkHAL:
    """
    Hardware Abstraction Layer for MAVLink drones (ArduPilot/PX4).
    """
    def __init__(self, connection_string):
        self.conn_str = connection_string
        self.vehicle = None
        
        # Internal state cache
        self._pos = np.zeros(3)
        self._vel = np.zeros(3)
        self._batt = 100.0
        
    async def connect(self):
        if not DRONEKIT_AVAILABLE: 
            print("[MavlinkHAL] Error: DroneKit not installed.")
            return False
            
        print(f"[MavlinkHAL] Connecting to {self.conn_str}...")
        self.vehicle = connect(self.conn_str, wait_ready=True, timeout=60)
        print("[MavlinkHAL] Connected.")
        
        self.vehicle.add_attribute_listener('location.local_frame', self._on_location)
        self.vehicle.add_attribute_listener('velocity', self._on_velocity)
        self.vehicle.add_attribute_listener('battery', self._on_battery)
        return True

    def _on_location(self, vehicle, name, msg):
        if msg: self._pos = np.array([msg.north, msg.east, msg.down])

    def _on_velocity(self, vehicle, name, msg):
        if msg: self._vel = np.array([msg[0], msg[1], msg[2]])

    def _on_battery(self, vehicle, name, msg):
        if msg: self._batt = msg.level

    def get_state(self) -> DroneState:
        mode = "UNKNOWN"
        armed = False
        heading = 0.0
        
        if self.vehicle:
            mode = self.vehicle.mode.name
            armed = self.vehicle.armed
            heading = math.radians(self.vehicle.heading)
            
        return DroneState(
            position_local=self._pos,
            velocity=self._vel,
            battery=self._batt,
            armed=armed,
            mode=mode,
            heading=heading,
            timestamp=time.time()
        )

    def land(self):
        if self.vehicle: self.vehicle.mode = VehicleMode("LAND")

    def return_to_launch(self):
        if self.vehicle: self.vehicle.mode = VehicleMode("RTL")

    def send_velocity_command(self, vel_cmd):
        if not self.vehicle or not DRONEKIT_AVAILABLE: return
        if self.vehicle.mode.name != "GUIDED": return

        type_mask = 0b0000111111000111 
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask, 0, 0, 0,
            vel_cmd[0], vel_cmd[1], vel_cmd[2], 0, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)