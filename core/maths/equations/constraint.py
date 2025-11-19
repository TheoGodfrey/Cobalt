import numpy as np
from .base import BaseEquation
from ..solver import SolverDebug

class ObstacleRepulsion(BaseEquation):
    def __init__(self, world_state, config):
        super().__init__(world_state, config)
        # Constraints config
        self.safety_margin = config.get('safety_margin', 5.0) # meters
        self.repulsion_gain = config.get('gain', 50.0)
        self.max_force = config.get('max_force', 20.0)
        
        # Placeholder for static obstacles (would come from a map service)
        # Format: {'center': [x,y,z], 'radius': r}
        self.obstacles = config.get('static_obstacles', [])

    def compute_gradient(self, drone_state):
        """
        Calculates repulsive gradient from obstacles.
        Returns [vx, vy, vz] command.
        """
        pos = drone_state.position_local # [x, y, z] (NED)
        total_repulsion = np.zeros(3)
        
        # 1. Static Obstacles (Spheres)
        for obs in self.obstacles:
            center = np.array(obs['center'])
            radius = obs['radius'] + self.safety_margin
            
            # Vector from obstacle to drone
            vec_to_drone = pos - center
            dist = np.linalg.norm(vec_to_drone)
            
            # If inside influence zone
            if dist < radius:
                # Direction away from obstacle
                if dist > 0:
                    direction = vec_to_drone / dist
                else:
                    direction = np.array([0, 0, -1]) # Panic up
                
                # Magnitude: 1/dist^2 (Standard Potential Field)
                # Normalized distance (0 at center, 1 at margin)
                # We want force to explode as dist -> 0
                
                # Ensure we don't divide by zero
                safe_dist = max(dist - obs['radius'], 0.1)
                
                magnitude = self.repulsion_gain / (safe_dist**2)
                total_repulsion += direction * magnitude

        # 2. Dynamic Obstacles (from Lidar via WorldState)
        # Assuming WorldState exposes a raw point cloud or nearest hit
        # For MVP, we check the simple 'nearest_obstacle_dist' if available
        # (This would be populated by the SafetyMonitor/HAL)
        
        # TODO: Integrate live Lidar point cloud repulsion here
        
        # 3. Floor/Ceiling Constraints (Soft Barriers)
        # Hard deck at z = 0 (altitude 0) -> Repel Up (Negative Z)
        # Ceiling at z = -120 (altitude 120) -> Repel Down (Positive Z)
        
        # Floor (Sea Surface)
        alt = -pos[2]
        if alt < self.safety_margin:
            # Push Up
            push = (self.safety_margin - alt) * self.repulsion_gain
            total_repulsion[2] -= push # Negative Z is Up
            
        # Ceiling
        ceiling = 120.0
        if alt > (ceiling - self.safety_margin):
            # Push Down
            push = (alt - (ceiling - self.safety_margin)) * self.repulsion_gain
            total_repulsion[2] += push # Positive Z is Down

        # Clamp
        norm = np.linalg.norm(total_repulsion)
        if norm > self.max_force:
            total_repulsion = (total_repulsion / norm) * self.max_force
            
        return total_repulsion
        
    def compute_gradient_debug(self, drone_state):
        grad = self.compute_gradient(drone_state)
        debug = SolverDebug(
            gradient_travel=np.zeros(3),
            gradient_search=grad, # Treat constraint as a high-priority search term
            gradient_wind=np.zeros(3),
            cost_value=np.linalg.norm(grad)
        )
        return grad, debug