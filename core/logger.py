"""
Structured Mission Logger

Writes JSON Lines format for post-flight analysis.
Critical for debugging sea trials and incident investigation.
"""
import json
import time
import os


class MissionLogger:
    """
    Minimal structured logger for mission events.
    
    Format: JSON Lines (.jsonl)
    Each line is a complete JSON object for easy parsing.
    
    Benefits:
    - Post-flight analysis with standard tools (jq, pandas)
    - Stream-safe (no corruption if process killed)
    - Human-readable
    """
    
    def __init__(self, drone_id, log_file="mission.jsonl"):
        self.drone_id = drone_id
        
        # Create logs directory if needed
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        
        self.file = open(log_file, 'a', buffering=1)  # Line buffered
        self.log_file = log_file
        
        print(f"[Logger] Logging to {log_file}")
    
    def log(self, event, data=None):
        """
        Log structured event.
        
        Args:
            event: Event type (string, e.g., "DETECTION", "STATE", "ERROR")
            data: Event data (dict, will be JSON serialized)
        """
        entry = {
            't': time.time(),           # Unix timestamp
            'id': self.drone_id,        # Drone identifier
            'event': event,             # Event type
            'data': data or {}          # Event payload
        }
        
        try:
            self.file.write(json.dumps(entry) + '\n')
            self.file.flush()  # Ensure written immediately
        except Exception as e:
            # Fallback to console if logging fails
            print(f"[Logger] ERROR: {e}")
            print(f"[Logger] Failed entry: {entry}")
    
    def close(self):
        """Close log file"""
        if self.file:
            self.file.close()
            print(f"[Logger] Closed {self.log_file}")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# Example usage and analysis tools

def analyze_log(log_file):
    """
    Quick analysis tool for post-flight review.
    
    Usage:
        from core.logger import analyze_log
        analyze_log('logs/scout_1_1234567890.jsonl')
    """
    events = []
    
    with open(log_file, 'r') as f:
        for line in f:
            events.append(json.loads(line))
    
    print(f"\n{'='*60}")
    print(f"Log Analysis: {log_file}")
    print(f"{'='*60}\n")
    
    # Event summary
    event_types = {}
    for e in events:
        event_types[e['event']] = event_types.get(e['event'], 0) + 1
    
    print("Event Summary:")
    for event, count in sorted(event_types.items(), key=lambda x: -x[1]):
        print(f"  {event:30s} {count:5d}")
    
    # Mission timeline
    if events:
        start_time = events[0]['t']
        end_time = events[-1]['t']
        duration = end_time - start_time
        
        print(f"\nMission Duration: {duration:.1f}s ({duration/60:.1f} min)")
    
    # Phase transitions
    print("\nPhase Transitions:")
    for e in events:
        if e['event'] == 'STATE':
            phase = e['data'].get('phase')
            if phase:
                t_rel = e['t'] - start_time
                print(f"  T+{t_rel:6.1f}s  Phase: {phase}")
    
    # Detections
    detections = [e for e in events if e['event'] == 'DETECTION']
    if detections:
        print(f"\nDetections: {len(detections)}")
        for d in detections[:5]:  # First 5
            t_rel = d['t'] - start_time
            label = d['data'].get('label', 'unknown')
            conf = d['data'].get('conf', 0)
            print(f"  T+{t_rel:6.1f}s  {label} ({conf:.2f})")
    
    # Errors/warnings
    errors = [e for e in events if e['event'] in ['ERROR', 'WARNING', 'EMERGENCY_KILL']]
    if errors:
        print(f"\n⚠️  Errors/Warnings: {len(errors)}")
        for err in errors:
            t_rel = err['t'] - start_time
            print(f"  T+{t_rel:6.1f}s  {err['event']}: {err['data']}")
    
    print(f"\n{'='*60}\n")
    
    return events


# Example: Stream logs in real-time
def tail_log(log_file):
    """
    Real-time log viewer (like 'tail -f').
    
    Usage:
        from core.logger import tail_log
        tail_log('logs/scout_1_1234567890.jsonl')
    """
    import time
    
    print(f"Tailing {log_file}...")
    print(f"{'Time':>10s} {'Event':20s} {'Data':40s}")
    print("-" * 72)
    
    with open(log_file, 'r') as f:
        # Seek to end
        f.seek(0, 2)
        
        while True:
            line = f.readline()
            if line:
                try:
                    entry = json.loads(line)
                    t = entry['t']
                    event = entry['event']
                    data_str = str(entry['data'])[:40]
                    
                    print(f"{t:10.1f} {event:20s} {data_str:40s}")
                except:
                    pass
            else:
                time.sleep(0.1)