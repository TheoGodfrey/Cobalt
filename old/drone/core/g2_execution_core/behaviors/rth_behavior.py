import asyncio

from .base import BaseBehavior
from ..mission_state import MissionStateEnum

class RTHBehavior(BaseBehavior):
    """
    Executes a simple Return-to-Hub task.
    Uses strategy for a home waypoint and HAL to fly there.
    """
    async def run(self):
        print(f"[RTHBehavior] Running...")
        if not self.strategy:
            print("[RTHBehavior] Error: Missing Strategy for home position.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        try:
            home_waypoint = await self.strategy.next_waypoint()
            print(f"[RTHBehavior] Returning to Hub at: ({home_waypoint.x}, {home_waypoint.y}, {home_waypoint.z})")
            
            arrival_success = await self.hal.goto(home_waypoint)
            if not arrival_success:
                print("[RTHBehavior] Failed to reach home waypoint. Attempting failsafe land.")
                self._increment_failure("navigation")
            else:
                self._reset_failures("navigation")
            
            print("[RTHBehavior] Arrived at Hub (or failed). Now landing...")
            
            await self.hal.land()
            
            print("[RTHBehavior] Landed.")
            self.mission_state.transition(MissionStateEnum.LANDING)
        except asyncio.CancelledError:
            print("[RTHBehavior] Canceled.")
        except Exception as e:
            print(f"[RTHBehavior] CRITICAL ERROR in run loop: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[RTHBehavior] Exiting run loop.")