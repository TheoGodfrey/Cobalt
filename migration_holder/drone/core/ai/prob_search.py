"""
ProbabilisticSearchManager (Item 5)

Implements the core logic from the patent:
- Maintains a spatiotemporal probability field p(x,y,t).
- Evolves the field with drift (e.g., ocean current).
- Updates the field with Bayesian logic based on sensor data.
- Generates optimal search waypoints.

This file was originally at `coordinator/prob_search.py`
and is now run locally by the Scout drone.
"""
import numpy as np
import math
from core.position import Position
from core.config_models import SearchAreaConfig, ProbSearchConfig

class ProbabilisticSearchManager:
    """Manages the probability grid for the MOB search."""
    
    def __init__(self, config: ProbSearchConfig, area: SearchAreaConfig):
        self.config = config
        self.area = area
        
        # Grid setup
        self.grid_size = config.grid_size
        self.cell_size = config.search_area_size_m / config.grid_size
        
        # The core probability field p(x,y)
        self.probability_grid = np.ones((self.grid_size, self.grid_size))
        self.total_search_area_m = config.search_area_size_m
        
        # Pre-calculate cell center positions (in meters)
        self.cell_centers_x, self.cell_centers_y = self._create_cell_centers()
        
        self.initialize_map()
        print(f"[ProbSearch] Initialized {self.grid_size}x{self.grid_size} grid.")
        print(f"[ProbSearch] Cell size: {self.cell_size:.1f}m. Total area: {self.total_search_area_m}m.")

    def _create_cell_centers(self):
        """Creates matrices of the world coordinates for each cell center."""
        half_area = self.total_search_area_m / 2.0
        # Vector of cell center coordinates
        coords = np.linspace(
            -half_area + self.cell_size / 2.0,
            half_area - self.cell_size / 2.0,
            self.grid_size
        )
        # Create 2D grids of coordinates
        return np.meshgrid(coords, coords)

    def initialize_map(self):
        """Initialize the map with a uniform (or Gaussian) prior."""
        # Start with a uniform probability
        self.probability_grid.fill(1.0 / (self.grid_size * self.grid_size))

    def get_next_search_waypoint(self) -> Position:
        """
        Finds the cell with the highest probability and returns its
        world-space position as the next search waypoint.
        """
        # Find the index of the max probability cell
        max_idx = np.unravel_index(
            np.argmax(self.probability_grid),
            self.probability_grid.shape
        )
        row, col = max_idx
        
        # Get the world coordinates for this cell
        # (Relative to the search area center)
        x = self.cell_centers_x[row, col] + self.area.x
        y = self.cell_centers_y[row, col] + self.area.y
        
        # Use a high altitude to maximize search radius
        z = self.config.search_altitude
        
        # Temporarily suppress this cell to encourage searching new areas
        self.probability_grid[row, col] *= 0.1 
        
        return Position(x=x, y=y, z=z)

    def update_map(self, drone_pos: Position, drone_altitude: float, has_detection: bool):
        """
        Bayesian update of the probability map based on a sensor observation.
        This is the core of the patent's logic.
        """
        if has_detection:
            # If a detection was made, we re-center the probability
            # In a real system, this is a complex update.
            # For now, we'll just log it.
            print(f"[ProbSearch] Detection reported at {drone_pos}. Map should be re-centered.")
            # self.probability_grid.fill(0.0)
            # ... (logic to create a new probability peak)
            return

        # --- No Detection (The common case) ---
        # 1. Calculate sensor radius based on altitude (Claim 5)
        # r(h) = r_max * h / (h + h_ref)
        sensor_radius = self.config.r_max * (drone_altitude / 
                         (drone_altitude + self.config.h_ref))
        
        # 2. Find all cells within the sensor radius
        # Calculate distance from drone to *all* cell centers at once
        dist_sq = (self.cell_centers_x - drone_pos.x)**2 + \
                  (self.cell_centers_y - drone_pos.y)**2
        
        sensor_radius_sq = sensor_radius**2
        
        # Create a boolean mask of cells inside the radius
        cells_in_radius = dist_sq < sensor_radius_sq
        
        # 3. Apply Bayesian Update
        # P(Target | No-Detect) = P(No-Detect | Target) * P(Target) / P(No-Detect)
        # P(No-Detect | Target) is the miss probability (e.g., 0.1)
        miss_prob = self.config.miss_probability
        
        # Update cells *inside* sensor radius
        self.probability_grid[cells_in_radius] *= miss_prob
        
        # 4. Normalize the entire grid so it sums to 1
        total_prob = np.sum(self.probability_grid)
        if total_prob > 0:
            self.probability_grid /= total_prob
        else:
            print("[ProbSearch] Warning: Probability grid collapsed. Re-initializing.")
            self.initialize_map()

    def evolve_map(self, dt: float):
        """
        Evolves the map over time to account for drift (e.g., ocean current).
        (∂p/∂t = -∇·(v_drift p))
        """
        # Calculate drift in pixels
        dx = int(self.config.drift_x_m_s * dt / self.cell_size)
        dy = int(self.config.drift_y_m_s * dt / self.cell_size)
        
        if dx != 0 or dy != 0:
            # Roll the numpy array, simulating the grid moving
            self.probability_grid = np.roll(self.probability_grid, shift=(dy, dx), axis=(0, 1))

    def confirm_target_at(self, pos: Position):
        """A target has been confirmed. Create a new probability peak here."""
        print(f"[ProbSearch] Target confirmed at {pos}. Locking map.")
        self.probability_grid.fill(0.0)
        
        # Find the cell index for this position
        half_area = self.total_search_area_m / 2.0
        col = int((pos.x - self.area.x + half_area) / self.cell_size)
        row = int((pos.y - self.area.y + half_area) / self.cell_size)
        
        # Clamp to grid size
        col = np.clip(col, 0, self.grid_size - 1)
        row = np.clip(row, 0, self.grid_size - 1)

        self.probability_grid[row, col] = 1.0