"""
Plugin: Obstacle Detector
A simple detector plugin that uses a Lidar sensor to find
the distance to the nearest obstacle.
"""
import asyncio
from typing import Dict, Any
# FIX: Change import from the 'sensors' package to the 'lidar' module
from ...g4_platform_interface.sensors.lidar import Lidar 

class ObstacleDetector:
    """
    A standalone detector that wraps a Lidar sensor.
    """
    
    def __init__(self, lidar_sensor: Lidar, config: Dict[str, Any]):
        """
        Initializes the detector.
        
        Args:
            lidar_sensor: An object adhering to the Lidar protocol.
            config: Configuration dictionary (e.g., for filtering)
        """
        self.lidar = lidar_sensor
        self.config = config
        print("[ObstacleDetector] Initialized.")

    async def detect(self) -> float:
        """
        Performs a detection cycle.
        
        Returns:
            The distance (float) to the nearest obstacle in meters.
        """
        try:
            distance = await self.lidar.get_scan()
            return distance
        except Exception as e:
            print(f"[ObstacleDetector] Error getting scan: {e}")
            return 999.0 # Return safe distance on error