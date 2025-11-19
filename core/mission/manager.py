import numpy as np
from core.maths.equations.rescue import UnifiedRescue
from core.maths.equations.energy import EnergyEfficientTransit
from core.maths.equations.transit import RapidTransit

class MissionManager:
    def __init__(self, plan, world, solver, safety, my_role="scout"):
        self.plan = plan
        self.world = world
        self.solver = solver
        self.safety = safety
        self.my_role = my_role
        self.current_phase_idx = -1
        self._dynamic_target_location = None

    # ... (start, set_role, advance_phase methods remain the same) ...
    def start(self): 
        self.advance_phase()

    def set_posture(self, new_posture):
        if self.current_posture != new_posture:
            print(f"[Manager] Switching Posture: {self.current_posture} -> {new_posture}")
            self.current_posture = new_posture
            self._reconfigure_solver()

    def advance_phase(self):
        self.current_phase_idx += 1
        if self.current_phase_idx >= len(self.plan['phases']):
            print("Mission Complete.")
            return

        phase = self.plan['phases'][self.current_phase_idx]
        print(f"Starting Phase: {phase['name']}")

        self._initialize_map(phase.get('map_initialization'))
        
        # Reset posture logic
        if 'default_behavior' in phase:
            self.current_posture = "default"
        else:
            self.current_posture = None 

        self._reconfigure_solver()

    def _resolve_location(self, config_loc):
        """
        Helper to resolve 'relative_to_hub' coordinates.
        Input: [x, y] or [x, y, z]
        Output: Absolute [x, y, z]
        """
        if config_loc is None: return None
        
        # Assume config_loc is relative offset [dx, dy, dz] from Hub
        # In a real mission file, you might add a flag like 'frame: relative'
        # For MVP, we treat ALL mission configs as relative to Hub (0,0,0)
        
        offset = np.array(config_loc)
        # Ensure 3D
        if len(offset) == 2: 
            offset = np.append(offset, 0.0)
            
        # Absolute Position = Hub Location + Offset
        absolute_pos = self.world.hub_location + offset
        return absolute_pos

    def _initialize_map(self, config):
        if not config: return
        strategy = config['strategy']
        
        if strategy == "InjectGaussian":
            # Resolve center relative to hub
            raw_center = config['config']['center']
            abs_center = self._resolve_location(raw_center)
            
            self.world.probability.inject_gaussian(
                abs_center, 
                config['config']['sigma']
            )
            
        elif strategy == "InjectUniformPolygon":
            # Resolve all vertices relative to hub
            raw_verts = config['config']['vertices']
            abs_verts = [self._resolve_location(v)[:2] for v in raw_verts] # Use 2D for map
            
            self.world.probability.inject_uniform_polygon(abs_verts)
            
        elif strategy == "MaintainCurrent":
            if self._dynamic_target_location is not None:
                self.world.probability.inject_gaussian(self._dynamic_target_location, 10.0)

    def _reconfigure_solver(self):
        if self.current_phase_idx < 0: return
        phase = self.plan['phases'][self.current_phase_idx]
        
        # Select Config
        config_to_use = {}
        if self.current_posture == "default" and 'default_behavior' in phase:
            config_to_use = phase['default_behavior']
        elif self.current_posture and 'postures' in phase:
            config_to_use = phase['postures'].get(self.current_posture, {})
        
        if not config_to_use: return

        # Resolve 'target_location' in config if it exists
        solver_config = config_to_use.get('config', {}).copy()
        if 'target_location' in solver_config:
            solver_config['target_location'] = self._resolve_location(solver_config['target_location'])

        # Load Equation
        eq_name = config_to_use.get('cost_equation', 'EnergyEfficientTransit')
        equation = self._get_eq_class(eq_name)(self.world, solver_config)
        self.solver.set_equation(equation)
        
        self.safety.update_constraints(config_to_use.get('constraints', {}))

    def _get_eq_class(self, name):
        if name == "UnifiedRescue": return UnifiedRescue
        if name == "EnergyEfficientTransit": return EnergyEfficientTransit
        if name == "RapidTransit": return RapidTransit
        raise ValueError(f"Unknown Equation: {name}")

    def check_triggers(self):
        if self.current_phase_idx < 0 or self.current_phase_idx >= len(self.plan['phases']): return
        phase = self.plan['phases'][self.current_phase_idx]
        
        if phase.get('trigger') == "target_found":
            best = self.world.targets.get_highest_confidence_target()
            if best and best.confidence > 0.8:
                print(f"Trigger Fired: Target Found ({best.id})")
                self._dynamic_target_location = best.position
                self.advance_phase()