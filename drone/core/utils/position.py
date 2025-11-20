"""
Defines simple data structures for 3D position.
"""

from dataclasses import dataclass

# FIX: Renamed to LocalPosition for clarity
@dataclass
class LocalPosition:
    """
    A simple dataclass to represent a 3D position in a local
    coordinate frame (e.g., NED - North, East, Down).
    Units are in meters.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __str__(self) -> str:
        return f"LocalPosition(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

# NEW: Added GlobalPosition dataclass
@dataclass
class GlobalPosition:
    """
    A dataclass to represent a 3D position in a global
    coordinate frame (WGS 84).
    """
    lat: float = 0.0  # Latitude in decimal degrees
    lon: float = 0.0  # Longitude in decimal degrees
    alt: float = 0.0  # Altitude in meters (relative to sea level or takeoff)
    
    def __str__(self) -> str:
        return f"GlobalPosition(lat={self.lat:.6f}, lon={self.lon:.6f}, alt={self.alt:.2f}m)"