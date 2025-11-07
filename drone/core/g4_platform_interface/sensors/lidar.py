"""
Sensor Interface: Lidar
Defines the abstract interface for a Lidar sensor.
"""
import asyncio
import random
from abc import ABC, abstractmethod
from typing import Protocol

class Lidar(Protocol):
    """Protocol for a Lidar sensor hardware interface."""
    
    @abstractmethod
    async def get_scan(self) -> float:
        """
        Get the closest distance from the Lidar.
        
        Returns:
            A float representing the closest distance in meters.
        """
        pass

    @abstractmethod
    async def start(self):
        """Start the sensor."""
        pass
        
    @abstractmethod
    async def stop(self):
        """Stop the sensor."""
        pass

class SimulatedLidar(Lidar):
    """
    Simulated Lidar implementation.
    This would be provided by the SimulatedController (HAL).
    """
    def __init__(self, sensor_id: int):
        self._sensor_id = sensor_id
        self._is_running = False
        print(f"[SimulatedLidar {self._sensor_id}] Initialized.")

    async def start(self):
        self._is_running = True
        print(f"[SimulatedLidar {self._sensor_id}] Started.")

    async def stop(self):
        self._is_running = False
        print(f"[SimulatedLidar {self._sensor_id}] Stopped.")

    async def get_scan(self) -> float:
        """
        Returns a simulated distance.
        Has a small chance of detecting a close obstacle.
        """
        if not self._is_running:
            await self.start()
            
        await asyncio.sleep(0.1) # Simulate 10Hz scan rate
        
        # 1% chance of detecting a close obstacle
        if random.random() < 0.01:
            distance = random.uniform(0.5, 1.9) # DANGER
        else:
            distance = 99.0 # Clear
            
        return distance