# shared/sensors/thermal_camera.py
import numpy as np
import cv2  # <-- Import OpenCV
import time

class ThermalCamera:
    """
    [REAL IMPLEMENTATION]
    Connects to a thermal camera as a simple video stream (e.g., RTSP or USB).
    This is the lightweight "Track 1" approach.
    """
    def __init__(self, drone):
        self.drone = drone
        self._gimbal_angle = -90 # Pointing straight down
        
        # --- THIS IS THE LINE TO CHANGE ---
        # Find your camera's stream URL or USB ID
        # 0 = First USB camera
        # "rtsp://192.168.1.100:554/stream" = Network camera
        self.stream_url = 0 
        
        print(f"[Camera] Connecting to video stream at: {self.stream_url}...")
        
        # Try to connect
        self.cap = cv2.VideoCapture(self.stream_url)
        
        # Give it a moment to connect
        time.sleep(1.0) 
        
        if not self.cap.isOpened():
            print(f"❌ [Camera] FAILED to open video stream.")
            self.cap = None
        else:
            print("[Camera] Thermal camera connected successfully.")
            
            # Read one frame to get the resolution
            ret, frame = self.cap.read()
            if ret:
                self.height, self.width, _ = frame.shape
                print(f"[Camera] Stream resolution: {self.width}x{self.height}")
            else:
                print(f"⚠️ [Camera] Could not read test frame.")

    def get_frame(self) -> np.ndarray:
        """
        Grabs a single frame from the video stream.
        """
        if self.cap is None:
            # Return a blank frame if connection failed
            return np.zeros((480, 640), dtype="uint8") 
            
        ret, frame = self.cap.read()
        
        if not ret:
            print("⚠️ [Camera] Failed to grab frame.")
            return np.zeros((self.height, self.width), dtype="uint8")
            
        # --- CONVERT TO 8-BIT GRAYSCALE ---
        # This is the "thermal image" (0-255) our detectors need
        # Most thermal streams are already grayscale or "false color"
        # We must convert to a single 8-bit channel.
        if len(frame.shape) == 3:
            # It's a color image (e.g., false-color palette)
            # Convert it to grayscale.
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            # It's already grayscale
            gray_frame = frame
            
        return gray_frame

    @property
    def gimbal_angle(self):
        """[STUB] Returns gimbal angle in degrees."""
        # In a real app, you'd get this from the drone SDK
        # e.g., return self.drone.get_gimbal_pitch()
        return self._gimbal_angle

    def __del__(self):
        """Release the camera when the object is destroyed."""
        if self.cap:
            print("[Camera] Releasing video stream.")
            self.cap.release()