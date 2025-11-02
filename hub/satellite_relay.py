"""
Satellite Relay
Simulates the uplink to Tier 3 (Global HQ).
"""

import asyncio

class SatelliteRelay:
    """
    Simulates a high-latency, low-bandwidth connection
    to a global HQ.
    """
    
    def __init__(self):
        self._is_connected = False
        print("[SatelliteRelay] Initialized.")

    async def connect(self):
        print("[SatelliteRelay] Establishing link...")
        await asyncio.sleep(5) # Simulate long connection time
        self._is_connected = True
        print("[SatelliteRelay] Link ESTABLISHED.")

    async def send_critical_update(self, update_type: str, data: dict):
        """Sends a high-priority, low-data message."""
        if not self._is_connected:
            print("[SatelliteRelay] Cannot send: Link down.")
            return False
            
        print(f"[SatelliteRelay] Sending update: {update_type}...")
        await asyncio.sleep(1.5) # Simulate send latency
        print(f"[SatelliteRelay] Update '{update_type}' SENT.")
        return True

    async def send_telemetry_summary(self, summary: dict):
        """Sends a periodic data summary."""
        if not self._is_connected:
            return False
            
        print("[SatelliteRelay] Sending periodic summary...")
        await asyncio.sleep(3) # Simulate larger data send
        print("[SatelliteRelay] Summary SENT.")
        return True

    async def run(self):
        """Main run loop for the relay."""
        await self.connect()
        while True:
            # Just keep the connection "alive"
            await asyncio.sleep(60)
            if self._is_connected:
                print("[SatelliteRelay] Link is active.")
