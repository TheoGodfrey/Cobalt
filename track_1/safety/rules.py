# shared/safety/rules.py
from typing import Dict, Any, Optional

class SafetyRule:
    """Base class for a safety rule."""
    def __init__(self):
        self.breach_message = "Safety rule breached."
        
    def check(self, telemetry: Dict[str, Any]) -> Optional[str]:
        """
        Checks if the rule is breached.
        Returns a breach message string if breached, None otherwise.
        """
        return None

    def get_safety_action(self) -> str:
        """
        Returns the required action ("HOVER" or "RTH").
        """
        return "HOVER" # Default safe action


class BatteryRule(SafetyRule):
    """Triggers if battery is critically low."""
    def __init__(self, min_percent: float = 20.0):
        super().__init__()
        self.min_percent = min_percent
        self.breach_message = f"CRITICAL: Battery below {self.min_percent}%!"
        
    def check(self, telemetry: Dict[str, Any]) -> Optional[str]:
        if telemetry.get("battery_percent", 100) < self.min_percent:
            return self.breach_message
        return None

    def get_safety_action(self) -> str:
        """If battery is low, we must return to home."""
        return "RTH"


class AltitudeRule(SafetyRule):
    """Triggers if drone flies too high (violates airspace)."""
    def __init__(self, max_altitude_m: float = 120.0):
        super().__init__()
        self.max_altitude_m = max_altitude_m
        self.breach_message = f"CRITICAL: Altitude above {self.max_altitude_m}m!"
        
    def check(self, telemetry: Dict[str, Any]) -> Optional[str]:
        if telemetry.get("altitude_m", 0) > self.max_altitude_m:
            return self.breach_message
        return None

    def get_safety_action(self) -> str:
        """If too high, stop and hover. Don't RTH."""
        return "HOVER"

# You could also add:
# class GeofenceRule(SafetyRule): ...
# class NoGpsRule(SafetyRule): ...