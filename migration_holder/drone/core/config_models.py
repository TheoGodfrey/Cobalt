"""
Pydantic models for validating the mission_config.yaml file.
"""
from typing import List, Tuple, Literal
from pydantic import BaseModel, Field

# --- NEW: Comms Config ---

class MqttConfig(BaseModel):
    host: str = "localhost"
    port: int = 1883

# --- GCS & Safety ---

class GcsConfig(BaseModel):
    host: str = "localhost"
    port: int = 8765

class SafetyConfig(BaseModel):
    min_obstacle_distance: float = 3.0

# --- Camera Intrinsics ---

class CameraIntrinsics(BaseModel):
    width: int
    height: int
    focal_length_x: float
    focal_length_y: float
    principal_point_x: float
    principal_point_y: float

# --- Camera Models ---

class ThermalCameraConfig(BaseModel):
    type: Literal["simulated", "flir_lepton", "seek_thermal"]
    resolution: Tuple[int, int]
    water_temp: float = 15.0
    ambient_temp: float = 20.0
    intrinsics: CameraIntrinsics

class VisualCameraConfig(BaseModel):
    type: Literal["simulated", "opencv", "picamera"]
    resolution: Tuple[int, int]
    intrinsics: CameraIntrinsics

class RecordingConfig(BaseModel):
    enabled: bool = True
    output_dir: str = "recordings"

class CameraConfig(BaseModel):
    thermal: ThermalCameraConfig
    visual: VisualCameraConfig
    recording: RecordingConfig

# --- Detection Models ---

class ThermalDetectionConfig(BaseModel):
    temp_threshold: float = 10.0
    min_area: int = 50
    max_area: int = 500
    min_confidence: float = 0.5

class VisualDetectionConfig(BaseModel):
    use_color: bool = True
    use_motion: bool = True
    min_confidence: float = 0.6

class FusionDetectionConfig(BaseModel):
    thermal_weight: float = 0.7
    visual_weight: float = 0.3
    fusion_threshold: float = 0.75
    max_position_error: int = 50

class DetectionConfig(BaseModel):
    method: Literal["fusion", "thermal_only", "visual_only"]
    thermal: ThermalDetectionConfig
    visual: VisualDetectionConfig
    fusion: FusionDetectionConfig

# --- Strategy Models ---

class SearchAreaConfig(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

class SearchStrategyConfig(BaseModel):
    algorithm: Literal["vertical_ascent", "random", "lawnmower"]
    area: SearchAreaConfig
    size: float = 1000.0

class FlightStrategyConfig(BaseModel):
    algorithm: Literal["precision_hover", "direct", "orbit"]

class StrategyConfig(BaseModel):
    search: SearchStrategyConfig
    flight: FlightStrategyConfig

# --- Other Component Models ---

class MissionConfig(BaseModel):
    max_search_iterations: int = 30
    search_timeout_seconds: int = 300

class LoggingConfig(BaseModel):
    log_dir: str = "logs"
    max_logs: int = 50

class HealthConfig(BaseModel):
    min_battery_preflight: float = 50.0
    min_battery_emergency: float = 20.0
    min_battery_patrol_rtl: float = 30.0
    max_heartbeat_latency: float = 5.0

class DroneConfig(BaseModel):
    id: str
    type: Literal["simulated", "real"]
    role: Literal["scout", "payload", "utility"]

class VerticalAscentConfig(BaseModel):
    max_altitude: float = 150.0
    step_size: float = 5.0

class PrecisionHoverConfig(BaseModel):
    altitude_offset: float = 2.0

class LawnmowerConfig(BaseModel):
    patrol_altitude: float = 40.0
    spacing: float = 50.0
    leg_length: float = 500.0
    num_legs: int = 10

class OrbitConfig(BaseModel):
    radius: float = 100.0
    speed: float = 10.0
    altitude_offset: float = 30.0

# --- NEW: Config block for Probabilistic AI (Item 5) ---

class ProbSearchConfig(BaseModel):
    """Configuration for the ProbabilisticSearchManager."""
    grid_size: int = 100 # Resolution of the probability grid (100x100)
    search_area_size_m: float = 2000.0 # Total area width/height in meters
    search_altitude: float = 100.0 # Altitude to send the drone to
    
    # Sensor model parameters (from patent)
    r_max: float = 500.0   # Max sensor radius at "infinite" altitude
    h_ref: float = 50.0    # Reference altitude for sensor model
    miss_probability: float = 0.1 # P(No-Detect | Target)
    
    # AI Control Loop parameters
    evolve_interval_s: float = 5.0 # How often to apply drift
    waypoint_interval_s: float = 10.0 # How long to wait at each waypoint
    drift_x_m_s: float = 0.5 # Ocean current simulation
    drift_y_m_s: float = 0.2

# --- Top-Level Settings Model ---

class Settings(BaseModel):
    """The root model for the entire mission_config.yaml."""
    mqtt: MqttConfig = Field(default_factory=MqttConfig)
    gcs: GcsConfig = Field(default_factory=GcsConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    mission: MissionConfig
    logging: LoggingConfig
    health: HealthConfig
    drones: List[DroneConfig]
    cameras: CameraConfig
    detection: DetectionConfig
    strategies: StrategyConfig
    vertical_ascent: VerticalAscentConfig = Field(default_factory=VerticalAscentConfig)
    precision_hover: PrecisionHoverConfig = Field(default_factory=PrecisionHoverConfig)
    lawnmower: LawnmowerConfig = Field(default_factory=LawnmowerConfig)
    orbit: OrbitConfig = Field(default_factory=OrbitConfig)
    prob_search: ProbSearchConfig = Field(default_factory=ProbSearchConfig) # NEW
