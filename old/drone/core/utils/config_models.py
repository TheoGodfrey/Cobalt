"""
Defines shared data models and configuration loaders.
"""

import yaml
from dataclasses import dataclass
from typing import Dict, Any, Optional

# FIX for Bug #16: Implement the CameraIntrinsics dataclass
# This class is required by drone/core/utils/navigation.py
@dataclass
class CameraIntrinsics:
    """
    Holds the intrinsic parameters for a camera, as used by
    the navigation.py geolocation logic.
    """
    focal_length_x: float
    focal_length_y: float
    principal_point_x: float
    principal_point_y: float
    width: int
    height: int

# FIX for Bug #6: Implement the load_config function
# This function is required by drone/main.py
def load_config(config_path: str) -> Dict[str, Any]:
    """
    Loads a YAML configuration file from the given path.
    
    Args:
        config_path: The file path to the .yaml config file.

    Returns:
        A dictionary containing the loaded configuration.
    """
    print(f"[Config] Loading configuration from: {config_path}")
    try:
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
        if config_data is None:
            return {}
        return config_data
    except FileNotFoundError:
        print(f"[Config] ERROR: Configuration file not found: {config_path}")
        raise
    except yaml.YAMLError as e:
        print(f"[Config] ERROR: Failed to parse YAML file {config_path}: {e}")
        raise
    except Exception as e:
        print(f"[Config] ERROR: An unexpected error occurred loading config: {e}")
        raise

# Example of a more complete config structure using Pydantic (as hinted by pyproject.toml)
# This is not required by the bug fix, but is good practice.

# from pydantic import BaseModel, Field
#
# class MqttConfig(BaseModel):
#     host: str = "localhost"
#     port: int = 1883
#
# class SafetyConfig(BaseModel):
#     battery_low_threshold: float = 25.0
#     geofence_path: str = "config/geofence.json"
#
# class DroneConfig(BaseModel):
#     drone_type: str = "simulated"
#     mqtt: MqttConfig = Field(default_factory=MqttConfig)
#     safety: SafetyConfig = Field(default_factory=SafetyConfig)
#
# def load_typed_config(config_path: str) -> DroneConfig:
#     """Loads and validates a config file into a Pydantic model."""
#     raw_config = load_config(config_path)
#     return DroneConfig(**raw_config)