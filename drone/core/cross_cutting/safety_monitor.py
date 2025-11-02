"""
Component 12: Safety Monitor
Cross-cutting observer that watches all components
"""
class SafetyMonitor:
    """Watches all components for hazardous conditions"""
    
    def __init__(self, config, mqtt):
        self.config = config
        self.mqtt = mqtt
        self.violations = []
        self.checkers = [
            BatteryChecker(config.battery),
            GeofenceChecker(config.geofence),
            StateConflictChecker(),
            CollisionChecker()
        ]
    
    async def monitor_loop(self):
        """Continuous safety monitoring"""
        while True:
            for checker in self.checkers:
                violation = await checker.check()
                if violation:
                    await self.handle_violation(violation)
            await asyncio.sleep(0.1)
    
    async def handle_violation(self, violation):
        """Execute safety action"""
        if violation.severity == "CRITICAL":
            await self.trigger_emergency_land()
        elif violation.severity == "HIGH":
            await self.trigger_rth()
        
        await self.mqtt.publish("fleet/safety/violation", violation)

class StateConflictChecker:
    """Prevents hazardous state combinations"""
    
    async def check(self):
        # Example: Can't be DELIVERING with battery < 20%
        if (mission_state.current_phase == "delivering" and 
            vehicle_state.battery < 20):
            return SafetyViolation(
                type="STATE_CONFLICT",
                message="Low battery during delivery",
                severity="HIGH"
            )