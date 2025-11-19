import numpy as np
import cv2
import time

class CameraSensor:
    """
    Wrapper for visual camera input (USB, CSI, or Mock).
    """
    def __init__(self, device_id=0, mode="mock"):
        self.mode = mode
        self.cap = None
        
        if self.mode != "mock":
            self.cap = cv2.VideoCapture(device_id)
            if not self.cap.isOpened():
                print(f"[Camera] Warning: Could not open device {device_id}. Falling back to mock.")
                self.mode = "mock"
            else:
                print(f"[Camera] Initialized Device {device_id}")
        else:
            print("[Camera] Initialized in MOCK mode.")

    def get_frame(self):
        if self.mode != "mock" and self.cap:
            ret, frame = self.cap.read()
            if ret: 
                return frame
        
        # --- MOCK IMAGE GENERATION ---
        # Returns a 640x480 image of blue water with an orange blob
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (100, 0, 0) # Blue background (BGR)
        
        # Draw a "Life Jacket" (International Orange) at a slight offset
        # Simulate target moving slowly
        t = time.time()
        cx = int(320 + 100 * np.sin(t * 0.5))
        cy = int(240 + 50 * np.cos(t * 0.5))
        
        # Ensure drawing is within bounds to prevent crash
        cx = np.clip(cx, 20, 620)
        cy = np.clip(cy, 20, 460)
        
        cv2.circle(img, (cx, cy), 15, (0, 165, 255), -1) 
        return img