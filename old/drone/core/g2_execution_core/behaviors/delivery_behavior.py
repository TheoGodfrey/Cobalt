import asyncio

from .base import BaseBehavior
from ..mission_state import MissionStateEnum

class DeliveryBehavior(BaseBehavior):
    """
    Executes a payload delivery task.
    Coordinates between a Strategy (to get drop point) and an
    Actuator (to release the payload).
    """
    async def run(self):
        print(f"[DeliveryBehavior] Running...")
        if not self.strategy or not self.actuator:
            print("[DeliveryBehavior] Error: Missing Strategy or Actuator.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        try:
            while self._running:
                if self.mission_state.current == MissionStateEnum.PAUSED:
                    await asyncio.sleep(1)
                    continue
                    
                drop_waypoint = await self.strategy.next_waypoint()
                print(f"[DeliveryBehavior] Moving to drop point: ({drop_waypoint.x}, {drop_waypoint.y}, {drop_waypoint.z})")

                arrival_success = await self.hal.goto(drop_waypoint)
                if not arrival_success:
                    print(f"[DeliveryBehavior] Failed to reach drop point.")
                    self._increment_failure("navigation") 
                    continue
                
                self._reset_failures("navigation")

                print(f"[DeliveryBehavior] At drop point, executing payload release...")
                release_success = await self.actuator.execute()
                
                if release_success:
                    print(f"[DeliveryBehavior] *** Payload Released Successfully! ***")
                    self._reset_failures("actuator")
                    self.mission_state.transition(MissionStateEnum.RETURNING)
                else:
                    print(f"[DeliveryBehavior] Payload release failed!")
                    self._increment_failure("actuator") 
                
                break # Delivery is usually a one-shot action
        except asyncio.CancelledError:
            print("[DeliveryBehavior] Canceled.")
        except Exception as e:
            print(f"[DeliveryBehavior] CRITICAL ERROR in run loop: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[DeliveryBehavior] Exiting run loop.")