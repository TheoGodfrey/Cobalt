"""
Mission Manager - Orchestrates mission phases with pause/resume support
Fixes: Missing pause/resume for manual override scenarios
"""
import time
import numpy as np


class MissionManager:
    """
    Mission Orchestrator (v2.0 architecture).
    
    Responsibilities:
    - Parse mission plan (phases, transitions)
    - Watch for transition events
    - Compose and assign cost functions to solver
    - Handle discrete logic (timers, events)
    
    NEW: Pause/resume support for manual override and emergency scenarios
    """
    
    def __init__(self, plan, world, solver, safety, my_role="scout"):
        self.plan = plan
        self.world = world
        self.solver = solver
        self.safety = safety
        self.my_role = my_role
        
        # Phase tracking
        self.phases = plan.get('phases', {})
        self.current_phase_idx = -1
        self.current_phase = None
        self.current_posture = None
        
        # NEW: Pause/resume state
        self.paused = False
        self.pause_reason = None
        self.pause_phase_idx = None  # Where we were when paused
        self.pause_time = None
        
        # Phase timing
        self.phase_start_time = None
        
        print(f"[Manager] Initialized for role={my_role}")
        print(f"[Manager] Phases available: {list(self.phases.keys())}")
    
    def start(self):
        """Begin mission execution"""
        if self.paused:
            print("[Manager] Cannot start while paused")
            return
        
        start_phase_name = self.plan.get('start_phase', list(self.phases.keys())[0])
        self.transition_to_phase(start_phase_name)
    
    def transition_to_phase(self, phase_name):
        """
        Transition to new phase.
        Reconfigures solver with new cost functions.
        """
        if phase_name not in self.phases:
            print(f"[Manager] ERROR: Unknown phase '{phase_name}'")
            return False
        
        print(f"[Manager] Transitioning: {self.current_phase} → {phase_name}")
        
        self.current_phase_idx = list(self.phases.keys()).index(phase_name)
        self.current_phase = phase_name
        self.phase_start_time = time.time()
        
        # Get phase configuration
        phase_config = self.phases[phase_name]
        
        # Find my role's task in this phase
        tasks = phase_config.get('tasks', {})
        my_task = tasks.get(self.my_role)
        
        if not my_task:
            print(f"[Manager] No task for {self.my_role} in {phase_name}")
            return False
        
        # Get behavior configuration
        behavior = my_task.get('default_behavior', {})
        self.current_posture = behavior.get('posture', 'COOPERATIVE')
        
        # Reconfigure solver with new cost function
        self._reconfigure_solver(behavior)
        
        return True
    
    def _reconfigure_solver(self, behavior=None):
        """
        Reconfigure solver with appropriate cost function for current phase.
        
        If behavior is None, reload from current phase.
        """
        if behavior is None:
            if self.current_phase is None:
                return
            
            phase_config = self.phases[self.current_phase]
            tasks = phase_config.get('tasks', {})
            my_task = tasks.get(self.my_role, {})
            behavior = my_task.get('default_behavior', {})
        
        # Clear existing equations
        self.solver.clear_equations()
        
        # Add cost function based on phase type
        equation_type = behavior.get('cost_equation', 'UnifiedRescue')
        
        print(f"[Manager] Loading cost function: {equation_type}")
        
        # Import appropriate equation class
        if equation_type == 'UnifiedRescue':
            from core.maths.equations.rescue import UnifiedRescue
            eq = UnifiedRescue(self.world, behavior)
            self.solver.add_equation(eq, weight=1.0)
        
        elif equation_type == 'RapidTransit':
            from core.maths.equations.transit import RapidTransit
            eq = RapidTransit(self.world, behavior)
            self.solver.add_equation(eq, weight=1.0)
        
        elif equation_type == 'DynamicTracking':
            from core.maths.equations.tracking import DynamicTracking
            eq = DynamicTracking(self.world, behavior)
            self.solver.add_equation(eq, weight=1.0)
        
        elif equation_type == 'EnergyEfficientTransit':
            from core.maths.equations.energy import EnergyEfficientTransit
            eq = EnergyEfficientTransit(self.world, behavior)
            self.solver.add_equation(eq, weight=1.0)
        
        else:
            print(f"[Manager] ERROR: Unknown equation type '{equation_type}'")
            return
        
        print(f"[Manager] Solver reconfigured for {self.current_phase}")
    
    def update(self, state):
        """
        Update mission state and check for transitions.
        Called every guidance loop iteration.
        """
        if self.paused:
            # Don't process transitions while paused
            return
        
        if self.current_phase is None:
            return
        
        # Check for phase transitions
        phase_config = self.phases[self.current_phase]
        transitions = phase_config.get('transitions', {})
        
        for trigger, targets in transitions.items():
            if self._check_trigger(trigger, state):
                # Trigger fired
                target_phase = targets.get(self.my_role) or targets.get('all_drones')
                if target_phase:
                    # Extract phase name from "goto:phase_name"
                    phase_name = target_phase.replace('goto:', '')
                    self.transition_to_phase(phase_name)
                    break
    
    def _check_trigger(self, trigger, state):
        """
        Check if a transition trigger has fired.
        
        Examples:
        - "on_event:target_found" → Check world.probability for high confidence
        - "on_state:battery_low" → Check state.battery
        - "on_timer:20min" → Check elapsed time
        """
        # Parse trigger format: "on_TYPE:VALUE"
        if not trigger.startswith('on_'):
            return False
        
        parts = trigger[3:].split(':', 1)
        if len(parts) != 2:
            return False
        
        trigger_type, trigger_value = parts
        
        if trigger_type == 'event':
            # Check for events (target_found, mission_complete, etc.)
            if trigger_value == 'target_found':
                # Check if probability model has high-confidence target
                if hasattr(self.world, 'probability'):
                    max_prob = np.max(self.world.probability.field)
                    return max_prob > 0.95
            
            elif trigger_value == 'target_lost':
                if hasattr(self.world, 'probability'):
                    max_prob = np.max(self.world.probability.field)
                    return max_prob < 0.3
        
        elif trigger_type == 'state':
            # Check vehicle state conditions
            if trigger_value == 'battery_low':
                return state.battery < 25.0
            
            elif trigger_value == 'altitude_reached':
                # Example: check if at desired altitude
                return abs(state.position_local[2] + 20) < 2.0  # Within 2m of -20m
        
        elif trigger_type == 'timer':
            # Check elapsed time in phase
            if self.phase_start_time:
                elapsed = time.time() - self.phase_start_time
                
                # Parse time string (e.g., "20min", "5s")
                if trigger_value.endswith('min'):
                    threshold = float(trigger_value[:-3]) * 60
                elif trigger_value.endswith('s'):
                    threshold = float(trigger_value[:-1])
                else:
                    threshold = float(trigger_value)
                
                return elapsed > threshold
        
        return False
    
    # ============================================================
    # NEW: Pause/Resume Support
    # ============================================================
    
    def pause_mission(self, reason="manual_override"):
        """
        Pause mission execution.
        
        Stores current phase for resume and clears solver equations
        so guidance outputs zeros (safe hover).
        
        Use cases:
        - Manual override (operator takes stick control)
        - Emergency condition detected by safety monitor
        - Communications loss (wait for reconnect)
        """
        if self.paused:
            print(f"[Manager] Already paused ({self.pause_reason})")
            return
        
        self.paused = True
        self.pause_reason = reason
        self.pause_phase_idx = self.current_phase_idx
        self.pause_time = time.time()
        
        # Clear solver equations → guidance outputs zero velocity
        self.solver.clear_equations()
        
        print(f"[Manager] ⏸️  PAUSED: {reason}")
        print(f"[Manager]    Phase: {self.current_phase}")
        print(f"[Manager]    Will resume to phase index: {self.pause_phase_idx}")
    
    def resume_mission(self):
        """
        Resume mission from paused state.
        
        Restores phase and reconfigures solver.
        """
        if not self.paused:
            print("[Manager] Not paused, cannot resume")
            return
        
        pause_duration = time.time() - self.pause_time
        
        print(f"[Manager] ▶️  RESUMING from {self.pause_reason}")
        print(f"[Manager]    Paused for: {pause_duration:.1f}s")
        
        # Restore phase
        self.current_phase_idx = self.pause_phase_idx
        phase_name = list(self.phases.keys())[self.current_phase_idx]
        self.current_phase = phase_name
        
        # Reconfigure solver with saved phase's cost function
        self._reconfigure_solver()
        
        # Clear pause state
        self.paused = False
        self.pause_reason = None
        self.pause_phase_idx = None
        
        print(f"[Manager] Resumed to phase: {self.current_phase}")
    
    def is_paused(self):
        """Check if mission is currently paused"""
        return self.paused
    
    def get_pause_info(self):
        """Get information about pause state"""
        if not self.paused:
            return None
        
        return {
            'reason': self.pause_reason,
            'duration': time.time() - self.pause_time,
            'paused_phase': list(self.phases.keys())[self.pause_phase_idx]
        }
    
    # ============================================================
    # Status Reporting
    # ============================================================
    
    def get_status(self):
        """Get current mission status for telemetry"""
        return {
            'phase': self.current_phase,
            'phase_idx': self.current_phase_idx,
            'posture': self.current_posture,
            'paused': self.paused,
            'pause_reason': self.pause_reason if self.paused else None
        }