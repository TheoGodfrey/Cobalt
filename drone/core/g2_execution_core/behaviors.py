"""
Component 4: Behaviors
Task execution with pure dependency injection
"""
from typing import Protocol

class SearchBehavior:
    """Generic search behavior that works with ANY detector/strategy/actuator"""
    
    def __init__(self, detector, strategy, actuator, flight_controller):
        # Pure dependency injection - NO creation logic
        self.detector = detector
        self.strategy = strategy
        self.actuator = actuator
        self.hal = flight_controller
    
    async def run(self):
        """Execute search task"""
        while not self.target_found:
            # Get next waypoint from strategy
            waypoint = self.strategy.next_waypoint()
            
            # Fly there
            await self.hal.go_to(waypoint)
            
            # Scan with detector
            detection = await self.detector.detect()
            
            if detection:
                # Signal with actuator
                await self.actuator.execute("SIGNAL_FOUND")
                self.target_found = True
                return detection

class BehaviorFactory:
    """Creates behaviors from JSON task definitions"""
    
    def create(self, task_definition: dict):
        behavior_class = self.get_behavior_class(task_definition['behavior'])
        
        # Create plugins based on JSON spec
        detector = get_detector(task_definition.get('detector'))
        strategy = get_strategy(task_definition.get('strategy'))
        actuator = get_actuator(task_definition.get('actuator'))
        
        # Inject all dependencies
        return behavior_class(
            detector=detector,
            strategy=strategy,
            actuator=actuator,
            flight_controller=self.flight_controller
        )