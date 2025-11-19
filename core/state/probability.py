import numpy as np
from scipy.ndimage import gaussian_filter, shift

class ProbabilityModel:
    def __init__(self, width_m=2000, height_m=2000, depth_m=100, resolution=10.0):
        self.res = resolution
        
        self.nz = int(depth_m / resolution)
        self.ny = int(height_m / resolution)
        self.nx = int(width_m / resolution)
        
        self.grid_shape = (self.nz, self.ny, self.nx)
        self.grid = np.zeros(self.grid_shape)
        
        self.DIFFUSION_XY = 0.1 
        self.DIFFUSION_Z = 0.01 

        # Initialize with uniform probability
        self.grid.fill(1.0)
        self._normalize()

    def _normalize(self):
        """Renormalizes the entire 3D grid so sum(P) = 1.0"""
        total = np.sum(self.grid)
        if total > 1e-9:
            self.grid /= total
        else:
            # --- FIX: Re-initialize if probability collapses ---
            self.grid.fill(1.0 / self.grid.size)

    def evolve(self, dt, drift_vector_ms=(0.0, 0.0, 0.0)):
        # --- FIX: Robustly handle both Tuples and Numpy Arrays ---
        drift = np.array(drift_vector_ms)
        
        sigma_xy = np.sqrt(2 * self.DIFFUSION_XY * dt) / self.res
        sigma_z = np.sqrt(2 * self.DIFFUSION_Z * dt) / self.res
        
        self.grid = gaussian_filter(self.grid, sigma=(sigma_z, sigma_xy, sigma_xy))
        
        # Check if the drift vector has any non-zero elements
        if np.any(drift != 0):
            sx = (drift[0] * dt) / self.res
            sy = (drift[1] * dt) / self.res
            sz = (drift[2] * dt) / self.res
            
            # Shift (z, y, x)
            self.grid = shift(self.grid, shift=(sz, sy, sx), mode='nearest')
            
        self._normalize()

    def update_from_sensor(self, x, y, z_drone, fov_angle_deg=60.0, confidence=0.5):
        cx = int(x / self.res)
        cy = int(y / self.res)
        cz_drone = int(z_drone / self.res)
        
        if not (0 <= cx < self.nx and 0 <= cy < self.ny):
            return

        tan_half_fov = np.tan(np.radians(fov_angle_deg / 2.0))
        
        z_start = min(cz_drone, self.nz - 1)
        z_end = 0 
        
        # Create coordinate grids 
        Y, X = np.ogrid[:self.ny, :self.nx]
        
        for z_idx in range(z_end, z_start + 1):
            h_above_layer = (cz_drone - z_idx) * self.res
            if h_above_layer <= 0: continue
            
            radius_m = h_above_layer * tan_half_fov
            r_px = radius_m / self.res
            
            if r_px < 0.5: 
                self.grid[z_idx, cy, cx] *= confidence
            else:
                # Update cone slice
                mask = (X - cx)**2 + (Y - cy)**2 <= r_px**2
                self.grid[z_idx][mask] *= confidence

        self._normalize()

    def inject_gaussian(self, center, sigma=50.0):
        cx = int(center[0] / self.res)
        cy = int(center[1] / self.res)
        cz = int(center[2] / self.res)
        
        if not (0 <= cx < self.nx and 0 <= cy < self.ny):
            return
        
        # Clamp Z to be safe
        cz = max(0, min(cz, self.nz - 1))

        Z, Y, X = np.ogrid[:self.nz, :self.ny, :self.nx]
        
        sigma_px = sigma / self.res
        dist_sq = (X - cx)**2 + (Y - cy)**2 + (Z - cz)**2
        gaussian = np.exp(-dist_sq / (2 * sigma_px**2))
        
        self.grid += gaussian
        self._normalize()

    def inject_uniform_polygon(self, vertices, z_range=(0, 5)):
        if not vertices: return

        min_x = int(min(v[0] for v in vertices) / self.res)
        max_x = int(max(v[0] for v in vertices) / self.res)
        min_y = int(min(v[1] for v in vertices) / self.res)
        max_y = int(max(v[1] for v in vertices) / self.res)
        
        min_z = int(z_range[0] / self.res)
        max_z = int(z_range[1] / self.res)
        
        min_x, max_x = max(0, min_x), min(self.nx, max_x)
        min_y, max_y = max(0, min_y), min(self.ny, max_y)
        min_z, max_z = max(0, min_z), min(self.nz, max_z)

        self.grid[min_z:max_z, min_y:max_y, min_x:max_x] += 0.1
        self._normalize()

    def get_active_cells(self, threshold=0.001):
        indices = np.argwhere(self.grid > threshold)
        results = []
        for z_idx, y_idx, x_idx in indices:
            prob = self.grid[z_idx, y_idx, x_idx]
            wx = (x_idx + 0.5) * self.res
            wy = (y_idx + 0.5) * self.res
            wz = (z_idx + 0.5) * self.res
            results.append((wx, wy, wz, prob))
        return results