"""
Asynchronous synchronized dual camera system (Thermal + Visual)
"""
import asyncio
import time
from dataclasses import dataclass
from typing import Optional, Tuple
from .base import ThermalFrame, VisualFrame, BaseCamera
# Assuming VideoRecorder is also refactored to have async methods
from core.recording.video_recorder import VideoRecorder 

# NEW: Custom exceptions for clarity
class CameraConnectionError(Exception):
    pass

class CameraCaptureError(Exception):
    pass

@dataclass
class DualFrame:
    """Synchronized frame from both cameras"""
    thermal: ThermalFrame
    visual: VisualFrame
    sync_timestamp: float
    time_delta: float  # Sync accuracy (ms)

class DualCameraSystem:
    """Manages thermal + visual camera system asynchronously."""
    
    def __init__(self, thermal_camera: BaseCamera, visual_camera: BaseCamera, recording_enabled: bool = True):
        """
        Initialize dual camera system
        
        Args:
            thermal_camera: An awaitable BaseCamera instance
            visual_camera: An awaitable BaseCamera instance
            recording_enabled: Enable video recording
        """
        self.thermal = thermal_camera
        self.visual = visual_camera
        self.recording_enabled = recording_enabled
        self.recorder: Optional[VideoRecorder] = None
        
        self.connected = False
        self.frame_count = 0
        
        # Sync parameters
        self.max_sync_delta = 0.1  # Max 100ms between frames
    
    # CHANGED: Now async
    async def connect(self) -> bool:
        """Connect both cameras concurrently."""
        print("[DualCamera] Connecting cameras...")
        
        try:
            # CHANGED: Run connections in parallel
            results = await asyncio.gather(
                self.thermal.connect(),
                self.visual.connect()
            )
            thermal_ok, visual_ok = results
            
            if not (thermal_ok and visual_ok):
                raise CameraConnectionError("One or more cameras failed to connect.")
            
            # Initialize video recorder if enabled
            if self.recording_enabled:
                self.recorder = VideoRecorder(
                    visual_resolution=self.visual.get_resolution(),
                    thermal_resolution=self.thermal.get_resolution()
                )
                await self.recorder.start()  # Assumes recorder.start() is async
            
            self.connected = True
            print("[DualCamera] Both cameras connected")
            return True
            
        except Exception as e:
            print(f"[DualCamera] Failed to connect: {e}")
            self.connected = False
            return False
    
    # CHANGED: Now async
    async def capture_synchronized(self) -> DualFrame:
        """Capture synchronized frame from both cameras concurrently."""
        if not self.connected:
            raise CameraCaptureError("Cameras not connected")
        
        try:
            sync_start = time.time()
            
            # CHANGED: Capture from both cameras in parallel
            thermal_frame, visual_frame = await asyncio.gather(
                self.thermal.capture(),  # Assumes camera.capture() is async
                self.visual.capture()
            )
            
            sync_timestamp = time.time()
            time_delta = abs(thermal_frame.timestamp - visual_frame.timestamp) * 1000  # ms
            
            self.frame_count += 1
            
            # Record if enabled
            if self.recorder:
                # Run recording in the background, don't block the capture loop
                asyncio.create_task(
                    self.recorder.write_frame(thermal_frame, visual_frame)
                )
            
            # Warn if sync is poor
            if time_delta > self.max_sync_delta * 1000:
                print(f"[DualCamera] Warning: Sync delta {time_delta:.1f}ms > {self.max_sync_delta*1000}ms")
            
            return DualFrame(
                thermal=thermal_frame,
                visual=visual_frame,
                sync_timestamp=sync_timestamp,
                time_delta=time_delta
            )
        
        except Exception as e:
            raise CameraCaptureError(f"Failed to capture frame: {e}")

    # CHANGED: Now async
    async def disconnect(self):
        """Disconnect both cameras and stop recording."""
        if self.recorder:
            await self.recorder.stop()  # Assumes recorder.stop() is async
        
        await asyncio.gather(
            self.thermal.disconnect(),
            self.visual.disconnect()
        )
        
        self.connected = False
        print("[DualCamera] Disconnected")