import asyncio

from .base import BaseBehavior
from ..mission_state import MissionStateEnum
from ...g3_capability_plugins.detectors.base import Detection

class SearchBehavior(BaseBehavior):
    """
    Executes a search task.
    Coordinates between a Strategy (to get waypoints) and a
    Detector (to find the target).
    
    Runs sensing and flight loops concurrently.
    """

    async def _handle_detection(self, detection: Detection):
        """Helper to process and publish a detection."""
        print(f"[SearchBehavior] *** Target Found! *** at {detection.position}")
        
        try:
            track_id = detection.track_id or f"trk_{int(detection.timestamp)}"
            
            payload = {
                "drone_id": self.drone_id,
                "detection": {
                    "class_label": detection.class_label,
                    "confidence": detection.confidence,
                    "position": detection.position,
                    "timestamp": detection.timestamp,
                    "track_id": track_id
                }
            }
            
            if self.mqtt.is_connected():
                await self.mqtt.publish(
                    f"fleet/detections/{self.drone_id}", 
                    payload
                )
            print(f"[SearchBehavior] Published detection {track_id} to hub.")
        except Exception as e:
            print(f"[SearchBehavior] WARNING: Failed to publish detection: {e}")

    async def run(self):
        """
        Runs the flight and sensing loops concurrently.
        """
        print(f"[SearchBehavior] Running (Concurrent Mode)...")
        if not self.strategy or not self.detector:
            print("[SearchBehavior] Error: Missing Strategy or Detector.")
            self.mission_state.transition(MissionStateEnum.ABORTED)
            return

        _target_found_event = asyncio.Event()

        async def _sensing_loop():
            """Continuously scans for targets."""
            print("[SearchBehavior] Sensing loop started...")
            try:
                while self._running and not _target_found_event.is_set():
                    if self.mission_state.current == MissionStateEnum.PAUSED:
                        await asyncio.sleep(1)
                        continue
                    
                    detection = await self.detector.detect()
                    
                    if detection and not _target_found_event.is_set():
                        await self._handle_detection(detection)
                        _target_found_event.set()
                        break
                    
                    await asyncio.sleep(0.05) # Fast scan rate
            except asyncio.CancelledError:
                print("[SearchBehavior] Sensing loop cancelled.")
            except Exception as e:
                print(f"[SearchBehavior] CRITICAL ERROR in sensing loop: {e}")
                self._increment_failure("detector_failure")
                _target_found_event.set()
            finally:
                print("[SearchBehavior] Sensing loop finished.")

        async def _flight_loop():
            """Continuously flies the search pattern."""
            print("[SearchBehavior] Flight loop started...")
            try:
                while self._running and not _target_found_event.is_set():
                    if self.mission_state.current == MissionStateEnum.PAUSED:
                        await asyncio.sleep(1)
                        continue
                    
                    waypoint = await self.strategy.next_waypoint()
                    print(f"[SearchBehavior] Moving to waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})")

                    goto_task = asyncio.create_task(self.hal.goto(waypoint))
                    event_wait_task = asyncio.create_task(_target_found_event.wait())
                    
                    done, pending = await asyncio.wait(
                        {goto_task, event_wait_task}, 
                        return_when=asyncio.FIRST_COMPLETED
                    )

                    if event_wait_task in done:
                        print("[SearchBehavior] Target found during flight. Cancelling goto.")
                        goto_task.cancel()
                        break 

                    arrival_success = await goto_task
                    
                    if not arrival_success:
                        print(f"[SearchBehavior] Failed to reach waypoint.")
                        self._increment_failure("navigation")
                        continue
                    
                    self._reset_failures("navigation")
            except asyncio.CancelledError:
                print("[SearchBehavior] Flight loop cancelled.")
            except Exception as e:
                print(f"[SearchBehavior] CRITICAL ERROR in flight loop: {e}")
                self._increment_failure("navigation")
                _target_found_event.set()
            finally:
                print("[SearchBehavior] Flight loop finished.")

        # --- Main execution of the concurrent loops ---
        try:
            print("[SearchBehavior] Starting concurrent flight and sensing loops.")
            sensing_task = asyncio.create_task(_sensing_loop())
            flight_task = asyncio.create_task(_flight_loop())
            
            done, pending = await asyncio.wait(
                {sensing_task, flight_task}, 
                return_when=asyncio.FIRST_COMPLETED
            )
            
            for task in pending:
                task.cancel()

            await asyncio.gather(sensing_task, flight_task, return_exceptions=True)
            
            if _target_found_event.is_set():
                if self.mission_state.current not in (MissionStateEnum.ABORTED, MissionStateEnum.NAVIGATION_FAILURE):
                    self.mission_state.transition(MissionStateEnum.CONFIRMING)
            
            print("[SearchBehavior] Concurrent run finished.")

        except asyncio.CancelledError:
            print("[SearchBehavior] Canceled.")
        except Exception as e:
            print(f"[SearchBehavior] CRITICAL ERROR in run setup: {e}")
            self.mission_state.transition(MissionStateEnum.ABORTED)
        finally:
            print("[SearchBehavior] Exiting run method.")