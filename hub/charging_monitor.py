"""
Charging Monitor
Monitors the status of drone charging pads at the docking station.
"""

import asyncio
import random
from enum import Enum
from typing import Dict, Callable, Set

class PadStatus(Enum):
    EMPTY = "empty"
    CHARGING = "charging"
    CHARGED = "charged"
    FAULT = "fault"

# Type alias for a listener
StatusListener = Callable[[int, PadStatus], None]

class ChargingMonitor:
    """
    Simulates monitoring of wireless charging pads.
    """
    
    def __init__(self, num_pads: int = 4):
        self.num_pads = num_pads
        self.pad_states: Dict[int, PadStatus] = {
            i: PadStatus.EMPTY for i in range(1, num_pads + 1)
        }
        self.listeners: Set[StatusListener] = set()
        print(f"[ChargingMonitor] Initialized with {num_pads} pads.")

    def add_listener(self, listener: StatusListener):
        self.listeners.add(listener)

    async def _update_pad_state(self, pad_id: int):
        """Simulates a state change for a single pad."""
        current_state = self.pad_states[pad_id]
        new_state = current_state
        
        if current_state == PadStatus.EMPTY:
            if random.random() < 0.1: # 10% chance drone lands
                new_state = PadStatus.CHARGING
                
        elif current_state == PadStatus.CHARGING:
            if random.random() < 0.2: # 20% chance it finishes charging
                new_state = PadStatus.CHARGED
            elif random.random() < 0.05: # 5% chance of fault
                new_state = PadStatus.FAULT
                
        elif current_state == PadStatus.CHARGED:
            if random.random() < 0.1: # 10% chance drone takes off
                new_state = PadStatus.EMPTY
                
        elif current_state == PadStatus.FAULT:
            if random.random() < 0.3: # 30% chance it clears
                new_state = PadStatus.EMPTY

        if new_state != current_state:
            self.pad_states[pad_id] = new_state
            print(f"[ChargingMonitor] Pad {pad_id} state -> {new_state.name}")
            # Notify listeners
            for listener in self.listeners:
                listener(pad_id, new_state)

    async def run(self):
        """Main monitoring loop."""
        print("[ChargingMonitor] Starting monitor loop.")
        while True:
            # Check a random pad every 2 seconds
            pad_to_check = random.randint(1, self.num_pads)
            await self._update_pad_state(pad_to_check)
            await asyncio.sleep(2)
