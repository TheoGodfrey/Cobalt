"""
Telemetry Logger for writing machine-readable logs (CSV).
"""
import csv
import time
from pathlib import Path
from datetime import datetime
from typing import List
from .drone import Drone, Telemetry  # <--- CORRECTED: Added 'Drone' import
from .cameras.base import Detection

class TelemetryLogger:
    """Logs drone state and detections to a CSV file."""
    
    def __init__(self, log_dir: str = "logs/telemetry"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = self.log_dir / f"telemetry_{timestamp}.csv"
        
        self.file_handle = open(self.log_file_path, 'w', newline='')
        self.writer = None
        self._write_header()
        
        print(f"[TelemetryLogger] Logging machine-readable data to: {self.log_file_path}")

    def _write_header(self):
        """Writes the CSV header row."""
        self.header = [
            'timestamp',
            'mission_state',
            'drone_id',
            'pos_x', 'pos_y', 'pos_z',
            'battery',
            'drone_state',
            'detection_count',
            'best_det_source',
            'best_det_confidence',
            'best_det_img_x',
            'best_det_img_y',
            'best_det_track_id'
        ]
        self.writer = csv.DictWriter(self.file_handle, fieldnames=self.header)
        self.writer.writeheader()
        self.file_handle.flush()

    async def log_snapshot(self, 
                           mission_state: str,
                           drone: 'Drone', # This type hint now resolves correctly
                           detections: List[Detection]):
        """
        Asynchronously writes a single snapshot of system state to the CSV.
        
        Args:
            mission_state: The current MissionPhase (as a string).
            drone: The Drone object.
            detections: List of current confirmed detections from the tracker.
        """
        timestamp = time.time()
        telemetry = drone.telemetry # Get latest telemetry
        
        best_det = None
        if detections:
            # Find the detection with the highest confidence
            best_det = max(detections, key=lambda d: d.confidence)

        row = {
            'timestamp': f"{timestamp:.3f}",
            'mission_state': mission_state,
            'drone_id': drone.id,
            'pos_x': f"{telemetry.position.x:.2f}",
            'pos_y': f"{telemetry.position.y:.2f}",
            'pos_z': f"{telemetry.position.z:.2f}",
            'battery': f"{telemetry.battery:.2f}",
            'drone_state': telemetry.state,
            'detection_count': len(detections),
            'best_det_source': best_det.source if best_det else 'N/A',
            'best_det_confidence': f"{best_det.confidence:.2f}" if best_det else 0.0,
            'best_det_img_x': best_det.position_image[0] if best_det else 0,
            'best_det_img_y': best_det.position_image[1] if best_det else 0,
            'best_det_track_id': best_det.metadata.get('track_id', 'N/A') if best_det else 'N/A'
        }
        
        # Write and flush to ensure data is saved
        self.writer.writerow(row)
        self.file_handle.flush()
        
    def close(self):
        """Closes the log file handle."""
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            print("[TelemetryLogger] Log file closed.")