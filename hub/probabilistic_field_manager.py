import numpy as np
import scipy.ndimage as ndimage
from typing import Tuple

class ProbabilityFieldManager:
    """
    Manages the spatiotemporal probability field p(x,y,t) for the fleet.
    This class lives on the Hub (in FleetCoordinator).
    """
    def __init__(self, width_m: int, height_m: int, resolution_m: int = 10):
        self.width_m = width_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        
        self.grid_width = width_m // resolution_m
        self.grid_height = height_m // resolution_m
        
        # Initialize a uniform probability grid
        self.grid = np.ones((self.grid_height, self.grid_width))
        self.normalize()
        
        print(f"[ProbField] Initialized {self.grid_width}x{self.grid_height} probability grid.")

    def normalize(self):
        """Ensures the sum of all probabilities in the grid is 1.0."""
        total = np.sum(self.grid)
        if total > 0:
            self.grid /= total
        else:
            # Failsafe: Re-initialize if probability collapses
            self.grid = np.ones((self.grid_height, self.grid_width))
            self.normalize()

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Converts world coordinates (meters) to grid indices."""
        # Assuming (0,0) world is (0,0) grid
        row = int(np.clip(y // self.resolution_m, 0, self.grid_height - 1))
        col = int(np.clip(x // self.resolution_m, 0, self.grid_width - 1))
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Converts grid indices to world coordinates (center of cell)."""
        x = (col + 0.5) * self.resolution_m
        y = (row + 0.5) * self.resolution_m
        return (x, y)

    def get_grid(self) -> np.ndarray:
        """Returns the current probability grid."""
        return self.grid

    def update_from_sensor(self, x_world: float, y_world: float, radius_m: float, detection_prob: float):
        """
        Updates the probability grid based on a sensor reading (Bayesian update).
        This implements Claim 12.
        
        Args:
            x_world: Center X coordinate of the sensor footprint (in meters).
            y_world: Center Y coordinate of the sensor footprint (in meters).
            radius_m: The sensing aperture radius r_i (in meters).
            detection_prob: The probability of this being a TRUE detection (e.g., 0.95 for a hit, 0.01 for a miss).
        """
        row_c, col_c = self.world_to_grid(x_world, y_world)
        radius_grid = radius_m / self.resolution_m
        
        # Create a mask for the circular sensor footprint
        y, x = np.ogrid[-row_c:self.grid_height-row_c, -col_c:self.grid_width-col_c]
        mask = x*x + y*y <= radius_grid*radius_grid
        
        # P(Target | Sensor) = P(Sensor | Target) * P(Target) / P(Sensor)
        # We update P(Target) [self.grid] using P(Sensor | Target) [detection_prob]
        
        # Update probabilities *within* the sensor footprint
        # P(Sensor | Target) = detection_prob
        self.grid[mask] *= detection_prob 
        
        # Update probabilities *outside* the sensor footprint
        # P(Sensor | ~Target) = (1 - detection_prob)
        self.grid[~mask] *= (1.0 - detection_prob)
        
        self.normalize()
        print(f"[ProbField] Update at ({x_world:.0f},{y_world:.0f}), r={radius_m}m, P(det)={detection_prob:.2f}")

    def evolve_field(self, dt_sec: float, drift_m_per_s: Tuple[float, float] = (0.0, 0.0)):
        """
        Evolves the field over time using diffusion (uncertainty growth) and drift.
        This implements Claim 8.
        """
        
        # 1. Apply diffusion (blurring) to simulate uncertainty growth
        # The 'sigma' represents how much the uncertainty spreads per second
        diffusion_sigma_per_sec = 0.1 # 10cm/s random walk
        sigma = (diffusion_sigma_per_sec * dt_sec) / self.resolution_m
        self.grid = ndimage.gaussian_filter(self.grid, sigma=sigma)
        
        # 2. Apply drift (shifting)
        drift_x_grid = (drift_m_per_s[0] * dt_sec) / self.resolution_m
        drift_y_grid = (drift_m_per_s[1] * dt_sec) / self.resolution_m
        self.grid = ndimage.shift(self.grid, shift=(drift_y_grid, drift_x_grid), mode='constant', cval=0)
        
        # 3. Renormalize after physics update
        self.normalize()
