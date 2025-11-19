from .probability import ProbabilityModel
from .wind import StaticWind, LearnedWindField
from .targets import TargetRegistry
import numpy as np

class WorldState:
    """
    The Container for Shared Reality.
    """
    def __init__(self, config=None):
        config = config or {}
        self.probability = ProbabilityModel()
        self.targets = TargetRegistry()
        wind_conf = config.get('wind_model', {'type': 'LearnedWindField'})
        self.wind = self._create_wind_model(wind_conf)
        
        # Store Hub Location (Default to 0,0,0 if not set)
        # This allows relative targeting: "500m North of Hub"
        self.hub_location = np.array([0.0, 0.0, 0.0])

    def set_hub_location(self, position):
        """Sets the global reference point for the Hub."""
        self.hub_location = np.array(position)

    def _create_wind_model(self, conf):
        model_type = conf.get('type')
        params = conf.get('config', {})
        print(f"[WorldState] Initializing Wind Model: {model_type}")
        if model_type == "StaticWind":
            return StaticWind(params)
        return LearnedWindField(params)

    def update_sensor_detection(self, detection_data):
        # FIX: Correct method call to TargetRegistry
        track_id = self.targets.update(
            detection_data['pos'], 
            detection_data['label'], 
            detection_data['confidence']
        )
        self.probability.update_from_sensor(
            x=detection_data['pos'][0],
            y=detection_data['pos'][1],
            radius=detection_data['radius'],
            confidence=detection_data['confidence']
        )
        return track_id

    def assign_roles_dynamically(self, fleet_states):
        # ... (Same as before)
        best_target = self.targets.get_best_target() # FIX: consistency check
        if not best_target or best_target.confidence < 0.8: return {}
        
        scores = {}
        for drone_id, state in fleet_states.items():
            dist = np.linalg.norm(state.position_local - best_target.position)
            scores[drone_id] = dist

        closest_drone = min(scores, key=scores.get)
        new_roles = {}
        for drone_id in fleet_states:
            new_roles[drone_id] = "delivery" if drone_id == closest_drone else "scout"
        return new_roles