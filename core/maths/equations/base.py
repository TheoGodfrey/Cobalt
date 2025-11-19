from abc import ABC, abstractmethod

class BaseEquation(ABC):
    def __init__(self, world_state, config):
        self.world = world_state
        self.config = config
    @abstractmethod
    def compute_gradient(self, drone_state): pass