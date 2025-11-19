import numpy as np
from scipy.ndimage import gaussian_filter, shift

class ProbabilityModel:
    def __init__(self, width_m=2000, height_m=2000, resolution=10.0):
        self.res = resolution
        self.grid_shape = (int(height_m/resolution), int(width_m/resolution))
        self.grid = np.zeros(self.grid_shape)
        self.DIFFUSION_COEFF = 0.1 

    def _normalize(self):
        total = np.sum(self.grid)
        if total > 1e-9: self.grid /= total

    def evolve(self, dt, drift_vector_ms=(0.0, 0.0)):
        sigma = np.sqrt(2 * self.DIFFUSION_COEFF * dt) / self.res
        self.grid = gaussian_filter(self.grid, sigma=sigma)
        if drift_vector_ms != (0.0, 0.0):
            sy = (drift_vector_ms[1] * dt) / self.res
            sx = (drift_vector_ms[0] * dt) / self.res
            self.grid = shift(self.grid, shift=(sy, sx), mode='nearest')
        self._normalize()

    def update_from_sensor(self, x, y, radius, confidence):
        cx, cy = int(x/self.res), int(y/self.res)
        r_px = int(radius/self.res)
        if not (0 <= cx < self.grid_shape[1] and 0 <= cy < self.grid_shape[0]): return
        
        Y, X = np.ogrid[:self.grid_shape[0], :self.grid_shape[1]]
        mask = (X - cx)**2 + (Y - cy)**2 <= r_px**2
        self.grid[mask] *= confidence
        self._normalize()

    def inject_gaussian(self, center, sigma=50.0):
        cx, cy = int(center[0]/self.res), int(center[1]/self.res)
        if not (0 <= cx < self.grid_shape[1] and 0 <= cy < self.grid_shape[0]): return
        Y, X = np.ogrid[:self.grid_shape[0], :self.grid_shape[1]]
        gaussian = np.exp(-((X-cx)**2 + (Y-cy)**2) / (2 * (sigma/self.res)**2))
        self.grid += gaussian
        self._normalize()

    def inject_uniform_polygon(self, vertices):
        min_x = int(min(v[0] for v in vertices) / self.res)
        max_x = int(max(v[0] for v in vertices) / self.res)
        min_y = int(min(v[1] for v in vertices) / self.res)
        max_y = int(max(v[1] for v in vertices) / self.res)
        self.grid[min_y:max_y, min_x:max_x] += 0.1
        self._normalize()
        
    def get_active_cells(self, threshold=0.001):
        indices = np.argwhere(self.grid > threshold)
        return [(c*self.res, r*self.res, self.grid[r,c]) for r,c in indices]