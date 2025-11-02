"""
Analyze mission logs for trends and statistics
"""

from pathlib import Path
import re
from collections import defaultdict
import sys

def find_log_directory():
    """Find the logs directory automatically"""
    # Try common locations
    possible_paths = [
        Path("logs"),                           # Current directory
        Path("v_0.2/scout_drone/logs"),        # From project root
        Path("../logs"),                        # One level up
        Path("../../logs"),                     # Two levels up
    ]
    
    for path in possible_paths:
        if path.exists():
            return path
    
    # If not found, return default
    return Path("logs")

def analyze_missions(log_dir: str = None):
    """Analyze all mission logs and generate statistics"""
    if log_dir is None:
        log_path = find_log_directory()
    else:
        log_path = Path(log_dir)
    
    if not log_path.exists():
        print(f"\n❌ Log directory not found: {log_path.absolute()}")
        print(f"\nTip: Run a mission first to create logs!")
        print(f"     cd v_0.2/scout_drone")
        print(f"     python main.py\n")
        return
    
    log_files = sorted(log_path.glob("mission_*.log"))
    
    if not log_files:
        print(f"\n⚠️  No mission logs found in: {log_path.absolute()}")
        print(f"\nTip: Run a mission first!")
        print(f"     cd v_0.2/scout_drone")
        print(f"     python main.py\n")
        return
    
    print(f"\n{'='*70}")
    print(f"MISSION LOG ANALYSIS")
    print(f"{'='*70}")
    print(f"Total missions: {len(log_files)}")
    print(f"Log directory: {log_path.absolute()}")
    print(f"{'='*70}\n")
    
    # Statistics
    stats = {
        'total_missions': len(log_files),
        'successful_detections': 0,
        'failed_detections': 0,
        'total_iterations': 0,
        'avg_iterations': 0,
        'strategies_used': defaultdict(int)
    }
    
    # Analyze each log
    for log_file in log_files:
        with open(log_file, 'r') as f:
            content = f.read()
            
            # Check if target was found
            if "Target found: Yes" in content:
                stats['successful_detections'] += 1
            elif "Target found: No" in content:
                stats['failed_detections'] += 1
            
            # Extract iterations
            match = re.search(r"Search iterations: (\d+)", content)
            if match:
                iterations = int(match.group(1))
                stats['total_iterations'] += iterations
            
            # Extract strategy
            match = re.search(r"Search strategy: (\w+)", content)
            if match:
                strategy = match.group(1)
                stats['strategies_used'][strategy] += 1
    
    # Calculate averages
    if stats['total_missions'] > 0:
        stats['avg_iterations'] = stats['total_iterations'] / stats['total_missions']
    
    # Print statistics
    print("STATISTICS:")
    print(f"  Successful detections: {stats['successful_detections']} ({stats['successful_detections']/stats['total_missions']*100:.1f}%)")
    print(f"  Failed detections: {stats['failed_detections']} ({stats['failed_detections']/stats['total_missions']*100:.1f}%)")
    print(f"  Average iterations per mission: {stats['avg_iterations']:.1f}")
    print(f"\nSTRATEGIES USED:")
    for strategy, count in stats['strategies_used'].items():
        print(f"  {strategy}: {count} missions")
    
    print(f"\n{'='*70}")
    print("RECENT MISSIONS:")
    print(f"{'='*70}")
    
    # Show last 5 missions
    for log_file in log_files[-5:]:
        print(f"\n{log_file.name}")
        with open(log_file, 'r') as f:
            # Find and print summary section
            content = f.read()
            if "MISSION SUMMARY" in content:
                summary_start = content.index("MISSION SUMMARY")
                summary_section = content[summary_start:summary_start+1000]
                # Print first few lines of summary
                lines = summary_section.split('\n')[2:10]
                for line in lines:
                    if line.strip() and not line.strip().startswith('='):
                        print(f"  {line}")
    
    print(f"\n{'='*70}\n")

def view_mission_index(log_dir: str = None):
    """Display the mission index file"""
    if log_dir is None:
        log_path = find_log_directory()
    else:
        log_path = Path(log_dir)
    
    index_file = log_path / "mission_index.txt"
    
    if not index_file.exists():
        print(f"\n❌ Mission index not found: {index_file.absolute()}")
        print(f"\nTip: Run a mission first to create the index!")
        print(f"     cd v_0.2/scout_drone")
        print(f"     python main.py\n")
        return
    
    print("\n" + "="*100)
    with open(index_file, 'r') as f:
        print(f.read())
    print("="*100 + "\n")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "index":
        view_mission_index()
    else:
        analyze_missions()