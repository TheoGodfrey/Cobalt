import numpy as np

class TravelTimeEstimator:
    def __init__(self, wind_field, airspeed=15.0):
        self.wind = wind_field
        self.airspeed = airspeed

    def estimate(self, start_pos, target_pos):
        vec = np.array(target_pos) - np.array(start_pos)
        dist = np.linalg.norm(vec)
        if dist < 1.0: return 0.0
        
        direction = vec / dist
        wind = self.wind.get_at(start_pos)
        gs = self.airspeed + np.dot(wind, direction)
        return dist / max(1.0, gs)