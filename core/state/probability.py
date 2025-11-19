import numpy as np
from scipy.ndimage import gaussian_filter, shift

class ProbabilityModel:
    def __init__(self, width_m=2000, height_m=2000, depth_m=100, resolution=10.0):
        """
        Initializes a 3D Voxel Grid for Target Probability.
        
        Args:
            width_m: Domain width (X-axis) in meters.
            height_m: Domain height (Y-axis) in meters.
            depth_m: Domain altitude (Z-axis) in meters.
            resolution: Voxel size in meters (cubic).
        """
        self.res = resolution
        
        # Grid Dimensions: [Depth (Z), Height (Y), Width (X)]
        self.nz = int(depth_m / resolution)
        self.ny = int(height_m / resolution)
        self.nx = int(width_m / resolution)
        
        self.grid_shape = (self.nz, self.ny, self.nx)
        self.grid = np.zeros(self.grid_shape)
        
        # Diffusion coefficients (Anisotropic: Fast horizontal, Slow vertical)
        # Units: m²/s
        self.DIFFUSION_XY = 0.1 
        self.DIFFUSION_Z = 0.01 

    def _normalize(self):
        """Renormalizes the entire 3D grid so sum(P) = 1.0"""
        total = np.sum(self.grid)
        if total > 1e-9:
            self.grid /= total
        else:
            # If mass is lost (e.g., drifted out), re-init uniform small prob or zero
            pass 

    def evolve(self, dt, drift_vector_ms=(0.0, 0.0, 0.0)):
        """
        Evolves the probability field via Advection-Diffusion.
        
        Args:
            dt: Time step in seconds.
            drift_vector_ms: (vx, vy, vz) wind/current vector in m/s.
        """
        # 1. Diffusion (Gaussian Blur)
        # Calculate sigma (std dev) for the filter kernel
        sigma_xy = np.sqrt(2 * self.DIFFUSION_XY * dt) / self.res
        sigma_z = np.sqrt(2 * self.DIFFUSION_Z * dt) / self.res
        
        # Apply 3D Gaussian filter
        self.grid = gaussian_filter(self.grid, sigma=(sigma_z, sigma_xy, sigma_xy))
        
        # 2. Advection (Shift)
        if drift_vector_ms != (0.0, 0.0, 0.0):
            # Calculate shift in grid cells
            sx = (drift_vector_ms[0] * dt) / self.res
            sy = (drift_vector_ms[1] * dt) / self.res
            sz = (drift_vector_ms[2] * dt) / self.res
            
            # Apply shift (nearest neighbor for speed, could use higher order)
            self.grid = shift(self.grid, shift=(sz, sy, sx), mode='nearest')
            
        self._normalize()

    def update_from_sensor(self, x, y, z_drone, fov_angle_deg=60.0, confidence=0.5):
        """
        Updates the grid based on a sensor observation (Negative or Positive).
        Projects a 3D Cone/Pyramid from the drone to the ground.
        
        For a "Miss" (Negative Information), we REDUCE probability in the cone.
        
        Args:
            x, y: Drone's 2D location (center of cone).
            z_drone: Drone's altitude (positive up, relative to sea level 0).
            fov_angle_deg: Camera Field of View.
            confidence: < 0.5 reduces prob (saw nothing), > 0.5 increases prob.
        """
        # Convert drone position to grid indices
        cx = int(x / self.res)
        cy = int(y / self.res)
        cz_drone = int(z_drone / self.res)
        
        # Bounds check
        if not (0 <= cx < self.nx and 0 <= cy < self.ny):
            return

        # Efficient 3D Cone Update (Simplified as a column for MVP performance, 
        # or a layer-by-layer radius check)
        
        # We iterate downwards from drone z to 0
        # Radius at relative depth d: r = d * tan(fov/2)
        tan_half_fov = np.tan(np.radians(fov_angle_deg / 2.0))
        
        # Limit Z to grid bounds
        z_start = min(cz_drone, self.nz - 1)
        z_end = 0 # Ground/Surface
        
        # Create coordinate grids for vectorized masking
        Y, X = np.ogrid[:self.ny, :self.nx]
        
        for z_idx in range(z_end, z_start + 1):
            # Height of drone above this layer
            h_above_layer = (cz_drone - z_idx) * self.res
            if h_above_layer < 0: continue # Should not happen given loop range
            
            # Radius of the cone at this layer
            radius_m = h_above_layer * tan_half_fov
            r_px = radius_m / self.res
            
            if r_px < 0.5: 
                # Just update single cell if very narrow
                self.grid[z_idx, cy, cx] *= confidence
            else:
                # Circular mask at this Z-layer
                mask = (X - cx)**2 + (Y - cy)**2 <= r_px**2
                self.grid[z_idx][mask] *= confidence

        self._normalize()

    def inject_gaussian(self, center, sigma=50.0):
        """
        Injects a 3D Gaussian blob. 
        If center Z is 0, it creates a "pancake" on the surface.
        """
        cx = int(center[0] / self.res)
        cy = int(center[1] / self.res)
        cz = int(center[2] / self.res)
        
        # Sanity check bounds
        if not (0 <= cx < self.nx and 0 <= cy < self.ny and 0 <= cz < self.nz):
            return

        # Create grids
        Z, Y, X = np.ogrid[:self.nz, :self.ny, :self.nx]
        
        # 3D Gaussian formula
        sigma_px = sigma / self.res
        dist_sq = (X - cx)**2 + (Y - cy)**2 + (Z - cz)**2
        gaussian = np.exp(-dist_sq / (2 * sigma_px**2))
        
        self.grid += gaussian
        self._normalize()

    def inject_uniform_polygon(self, vertices, z_range=(0, 5)):
        """
        Injects uniform probability into a polygon area, usually at the surface (z=0).
        """
        # Simplified: Bounding box injection for MVP
        min_x = int(min(v[0] for v in vertices) / self.res)
        max_x = int(max(v[0] for v in vertices) / self.res)
        min_y = int(min(v[1] for v in vertices) / self.res)
        max_y = int(max(v[1] for v in vertices) / self.res)
        
        min_z = int(z_range[0] / self.res)
        max_z = int(z_range[1] / self.res)
        
        # Clip
        min_x, max_x = max(0, min_x), min(self.nx, max_x)
        min_y, max_y = max(0, min_y), min(self.ny, max_y)
        min_z, max_z = max(0, min_z), min(self.nz, max_z)

        self.grid[min_z:max_z, min_y:max_y, min_x:max_x] += 0.1
        self._normalize()

    def get_active_cells(self, threshold=0.001):
        """
        Returns a list of (x, y, z, prob) for all cells above threshold.
        Used by the Solver to compute gradients.
        """
        indices = np.argwhere(self.grid > threshold)
        results = []
        for z_idx, y_idx, x_idx in indices:
            prob = self.grid[z_idx, y_idx, x_idx]
            # Convert back to world meters
            wx = (x_idx + 0.5) * self.res
            wy = (y_idx + 0.5) * self.res
            wz = (z_idx + 0.5) * self.res
            results.append((wx, wy, wz, prob))
        return results