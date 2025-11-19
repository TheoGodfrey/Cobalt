from .probability import ProbabilityModel
from .wind import StaticWind, LearnedWindField
from .targets import TargetRegistry
import numpy as np

class WorldState:
    """
    The Container for Shared Reality (3D Edition).
    """
    def __init__(self, config=None):
        config = config or {}
        
        # Initialize 3D Probability Model
        # Default to 2km x 2km area, 100m altitude ceiling
        self.probability = ProbabilityModel(
            width_m=config.get('width', 2000), 
            height_m=config.get('height', 2000), 
            depth_m=config.get('depth', 100),
            resolution=config.get('resolution', 10.0)
        )
        
        self.targets = TargetRegistry()
        
        wind_conf = config.get('wind_model', {'type': 'LearnedWindField'})
        self.wind = self._create_wind_model(wind_conf)
        
        # Hub location 
        self.hub_location = np.array([0.0, 0.0, 0.0])

    def set_hub_location(self, position):
        self.hub_location = np.array(position)

    def _create_wind_model(self, conf):
        model_type = conf.get('type')
        params = conf.get('config', {})
        print(f"[WorldState] Initializing Wind Model: {model_type}")
        if model_type == "StaticWind":
            return StaticWind(params)
        return LearnedWindField(params)

    def update_sensor_detection(self, detection_data):
        """
        Ingests a detection (or miss) into the world state.
        """
        pos = detection_data['pos']
        
        # If it's a positive detection
        if detection_data['confidence'] > 0.6: # Threshold
             track_id = self.targets.update(
                pos, 
                detection_data['label'], 
                detection_data['confidence']
            )
             # Also reinforce probability map at that location (Gaussian blob)
             self.probability.inject_gaussian(pos, sigma=20.0)
             return track_id
        
        # If it's a negative detection (Miss), handled by main loop calling
        # self.probability.update_from_sensor(...) explicitly 
        # because that requires drone altitude/FOV info which might not be in detection_data
        return None
        
    # ... assign_roles_dynamically remains same ...