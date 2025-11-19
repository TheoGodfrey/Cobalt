# core/types.py
import time
import numpy as np
from pydantic import BaseModel, Field, ConfigDict
from typing import List, Optional, Any

class Vector3(BaseModel):
    """Strict 3D Vector validator to prevent Tuple/List confusion."""
    x: float
    y: float
    z: float

    model_config = ConfigDict(arbitrary_types_allowed=True)

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    @staticmethod
    def from_numpy(arr: np.ndarray):
        if len(arr) != 3:
            raise ValueError(f"Expected shape (3,), got {arr.shape}")
        return Vector3(x=arr[0], y=arr[1], z=arr[2])

class DroneState(BaseModel):
    """Standardized Drone State across HAL, Solver, and Comms."""
    position_local: np.ndarray = Field(..., description="NED Position [x, y, z]")
    velocity: np.ndarray = Field(..., description="NED Velocity [vx, vy, vz]")
    battery: float = Field(..., ge=0.0, le=100.0)
    armed: bool
    mode: str
    heading: float # Radians
    timestamp: float = Field(default_factory=time.time)

    model_config = ConfigDict(arbitrary_types_allowed=True)

class TelemetryMessage(BaseModel):
    """Network packet structure for fleet telemetry."""
    id: str
    pos: List[float] # JSON friendly
    vel: List[float]
    hdg: float
    batt: float
    mode: str
    phase: int
    t: float = Field(default_factory=time.time)