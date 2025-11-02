"""
Defines a simple data structure for 3D position.
"""

from dataclasses import dataclass

# FIX for Bug #17: Implement the Position class
# This class is imported and used by drone/core/utils/navigation.py
@dataclass
class Position:
    """
    A simple dataclass to represent a 3D position in a local
    coordinate frame (e.g., NED - North, East, Down).
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __str__(self) -> str:
        return f"Position(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"