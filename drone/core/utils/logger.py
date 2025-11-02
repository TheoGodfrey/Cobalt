"""
Enhanced mission logger with incremental log files
"""

import time
from pathlib import Path
from datetime import datetime

class MissionLogger:
    """Enhanced mission logger with timestamped log files"""
    
    def __init__(self, log_dir: str = "logs", max_logs: int = 0, drone_id: str = "drone"): # <-- FIX: Added drone_id
        """
        Initialize logger with log directory
        
        Args:
            log_dir: Directory to store log files
            max_logs: Maximum number of logs to keep (0 = unlimited)
            drone_id: The ID of the drone for log naming
        """
        self.log_dir = Path(log_dir)
        self.max_logs = max_logs
        self.drone_id = drone_id # <-- ADDED
        
        # Create logs directory if it doesn't exist
        self.log_dir.mkdir(exist_ok=True)
        
        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mission_number = self._get_next_mission_number()
        # --- FIX: Use drone_id in log file name ---
        self.log_file = self.log_dir / f"{self.drone_id}_mission_{mission_number:04d}_{timestamp}.log"
        
        # Create summary index file path
        # --- FIX: Use drone_id in index file name ---
        self.index_file = self.log_dir / f"{self.drone_id}_mission_index.txt"
        
        # Initialize log file with header
        self._write_header()
        
        # Clean old logs if needed
        if self.max_logs > 0:
            self._cleanup_old_logs()
    
    def _get_next_mission_number(self) -> int: # <-- FIX: No longer needs drone_id
        """Get the next sequential mission number"""
        # --- FIX: Use self.drone_id ---
        log_files = sorted(self.log_dir.glob(f"{self.drone_id}_mission_*.log"))
        
        if not log_files:
            return 1
        
        # Extract mission numbers from filenames
        try:
            last_file = log_files[-1].stem  # Get filename without extension
            # Extract number from "drone-id_mission_NNNN_timestamp" format
            mission_num = int(last_file.split('_')[2]) # <-- FIX: Index is 2
            return mission_num + 1
        except (IndexError, ValueError):
            # Fallback for old format
            try:
                mission_num = int(last_file.split('_')[1])
                return mission_num + 1
            except (IndexError, ValueError):
                return 1 # Default
    
    def _write_header(self):
        """Write log file header with metadata"""
        header = f"""
{'='*70}
MISSION LOG (Drone: {self.drone_id})
{'='*70}
Log File: {self.log_file.name}
Start Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
{'='*70}

"""
        with open(self.log_file, 'w') as f:
            f.write(header)
    
    def _cleanup_old_logs(self):
        """Remove old log files if we exceed max_logs"""
        # --- FIX: Use self.drone_id ---
        log_files = sorted(self.log_dir.glob(f"{self.drone_id}_mission_*.log"))
        
        if len(log_files) > self.max_logs:
            # Remove oldest logs
            files_to_remove = log_files[:-self.max_logs]
            for old_log in files_to_remove:
                old_log.unlink()
                print(f"Removed old log: {old_log.name}")
    
    def log(self, message: str, level: str = "info"):
        """Log a message with the specified level"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log_line = f"[{timestamp}] [{level.upper()}] {message}"
        
        # Write to file
        with open(self.log_file, "a") as f:
            f.write(log_line + "\n")
        
        # Also print to console with colors (optional)
        colors = {
            'error': '\033[91m',    # Red
            'warning': '\033[93m',  # Yellow
            'info': '\033[0m',      # Default
            'debug': '\033[90m'     # Gray
        }
        reset = '\033[0m'
        
        color = colors.get(level, colors['info'])
        print(f"{color}{log_line}{reset}")
    
    def log_summary(self, summary_data: dict):
        """Log mission summary and update index"""
        # Write summary to current log
        self.log("", "info")
        self.log("="*70, "info")
        self.log("MISSION SUMMARY", "info")
        self.log("="*70, "info")
        
        for key, value in summary_data.items():
            self.log(f"{key}: {value}", "info")
        
        self.log("="*70, "info")
        
        # Update index file
        self._update_index(summary_data)
    
    def _update_index(self, summary_data: dict):
        """Update the mission index file"""
        index_line = (
            f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | "
            f"{self.log_file.name} | "
            f"Target: {summary_data.get('Target found', 'N/A')} | "
            f"Iterations: {summary_data.get('Search iterations', 'N/A')} | "
            f"Battery: {summary_data.get('Final battery', 'N/A')}\n"
        )
        
        # Create index if it doesn't exist
        if not self.index_file.exists():
            with open(self.index_file, 'w') as f:
                f.write(f"MISSION INDEX (Drone: {self.drone_id})\n")
                f.write("="*100 + "\n")
                f.write(f"{'Timestamp':<20} | {'Log File':<40} | {'Target':<10} | {'Iterations':<12} | {'Battery':<10}\n")
                f.write("="*100 + "\n")
        
        # Append to index
        with open(self.index_file, 'a') as f:
            f.write(index_line)
    
    def get_log_path(self) -> Path:
        """Get the current log file path"""
        return self.log_file