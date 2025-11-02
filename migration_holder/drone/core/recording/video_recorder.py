"""
Video recording for mission documentation
"""
import cv2
import time
from pathlib import Path
from datetime import datetime
import numpy as np

class VideoRecorder:
    """Records synchronized thermal + visual video"""
    
    def __init__(self, visual_resolution: tuple, thermal_resolution: tuple, output_dir: str = "recordings"):
        """
        Initialize video recorder
        
        Args:
            visual_resolution: Visual camera resolution (width, height)
            thermal_resolution: Thermal camera resolution (width, height)
            output_dir: Directory to save recordings
        """
        self.visual_resolution = visual_resolution
        self.thermal_resolution = thermal_resolution
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.visual_file = self.output_dir / f"visual_{timestamp}.avi"
        self.thermal_file = self.output_dir / f"thermal_{timestamp}.avi"
        
        # Video writers
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.visual_writer = cv2.VideoWriter(
            str(self.visual_file),
            fourcc,
            10.0,  # FPS
            visual_resolution
        )
        self.thermal_writer = cv2.VideoWriter(
            str(self.thermal_file),
            fourcc,
            10.0,
            thermal_resolution
        )
        
        self.recording = False
        self.frame_count = 0
        
        print(f"[Recorder] Initialized")
        print(f"  Visual: {self.visual_file}")
        print(f"  Thermal: {self.thermal_file}")
    
    def start(self):
        """Start recording"""
        self.recording = True
        print("[Recorder] Recording started")
    
    def write_frame(self, thermal_frame, visual_frame):
        """Write synchronized frame pair"""
        if not self.recording:
            return
        
        # Convert thermal to colormap for visualization
        thermal_normalized = cv2.normalize(
            thermal_frame.temperature_array,
            None,
            0,
            255,
            cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )
        thermal_colored = cv2.applyColorMap(thermal_normalized, cv2.COLORMAP_JET)
        
        # Resize thermal to match aspect ratio if needed
        thermal_resized = cv2.resize(thermal_colored, self.thermal_resolution)
        
        # Write frames
        self.thermal_writer.write(thermal_resized)
        self.visual_writer.write(cv2.cvtColor(visual_frame.image, cv2.COLOR_RGB2BGR))
        
        self.frame_count += 1
    
    def stop(self):
        """Stop recording and close files"""
        if not self.recording:
            return
        
        self.recording = False
        self.visual_writer.release()
        self.thermal_writer.release()
        
        print(f"[Recorder] Recording stopped - {self.frame_count} frames saved")
        print(f"  Visual: {self.visual_file}")
        print(f"  Thermal: {self.thermal_file}")