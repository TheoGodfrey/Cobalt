import asyncio

from .base import BaseBehavior
from ..mission_state import MissionStateEnum

class SacrificialDeliveryBehavior(BaseBehavior):
    """
    Executes a one-way delivery task that ends in landing.
    It flies to the single waypoint provided by its strategy
    (e.g., an OffsetTargetStrategy) and then lands.
    This behavior does not return.
    """
    async def run(self):
        print(f"[SacrificialDeliveryBehavior] Running...")
        if not self.strategy:
            print("[SacrificialDeliveryBehavior] Error: Missing Strategy.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        try:
            waypoint = await self.strategy.next_waypoint()
            print(f"[SacrificialDeliveryBehavior] Flying to one-way drop point: ({waypoint.x}, {waypoint.y}, {waypoint.z})")
            
            arrival_success = await self.hal.goto(waypoint)
            if not arrival_success:
                print(f"[SacrificialDeliveryBehavior] Failed to reach drop point.")
                self._increment_failure("navigation")
            
            print(f"[SacrificialDeliveryBehavior] At target. Landing in water...")
            
            await self.hal.land()
            
            print(f"[SacrificialDeliveryBehavior] Landed. Mission complete.")
            self.mission_state.transition(MissionStateEnum.MISSION_COMPLETE)
            
        except asyncio.CancelledError:
            print("[SacrificialDeliveryBehavior] Canceled.")
        except Exception as e:
            print(f"[SacrificialDeliveryBehavior] CRITICAL ERROR in run loop: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[SacrificialDeliveryBehavior] Exiting run loop.")