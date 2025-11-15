"""
Mission File Loader
Responsible for reading mission definition files (JSON) from disk.
"""

import json
from pathlib import Path
from typing import Dict, Any

class MissionLoadError(Exception):
    """Custom exception for errors during mission file loading."""
    pass

def load_mission_file(file_path: Path) -> Dict[str, Any]:
    """
    Reads a JSON mission file from the specified path.

    Args:
        file_path: The Path object pointing to the .json mission file.

    Returns:
        A dictionary containing the raw, unparsed mission data.

    Raises:
        MissionLoadError: If the file is not found, is not a file,
                          or contains invalid JSON.
    """
    if not file_path.exists():
        raise MissionLoadError(f"Mission file not found: {file_path}")
    if not file_path.is_file():
        raise MissionLoadError(f"Path is not a file: {file_path}")

    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    except json.JSONDecodeError as e:
        raise MissionLoadError(f"Failed to parse JSON in {file_path}: {e}")
    except IOError as e:
        raise MissionLoadError(f"Failed to read file {file_path}: {e}")
