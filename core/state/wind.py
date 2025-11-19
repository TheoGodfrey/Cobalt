from abc import ABC, abstractmethod
import numpy as np
import time

class BaseWindModel(ABC):
    @abstractmethod
    def update(self, position, velocity): pass
    @abstractmethod
    def get_at(self, position): pass

class LearnedWindField(BaseWindModel):
    def __init__(self, config=None):
        self.obs = [] 
        self.sigma_s = 500.0
        self.sigma_t = 300.0

    def update(self, pos, vel):
        self.obs.append({'p': np.array(pos), 'v': np.array(vel), 't': time.time()})
        if len(self.obs) > 500: self.obs.pop(0)

    def get_at(self, pos):
        if not self.obs: return np.array([0.,0.,0.])
        pos = np.array(pos)
        now = time.time()
        w_sum, tot = np.zeros(3), 1e-6
        for o in self.obs:
            d2 = np.sum((pos - o['p'])**2)
            t2 = (now - o['t'])**2
            w = np.exp(-d2/(2*self.sigma_s**2)) * np.exp(-t2/(2*self.sigma_t**2))
            w_sum += o['v'] * w
            tot += w
        return w_sum / tot