"""
Tier 2 Hub: Satellite Relay (Uplink)

This is a "non-thinking" component as described in the COBALT document.
It listens for high-priority local P2P messages and "uplinks" them
to the Tier 3 Global HQ via a simulated satellite topic.

It also serves as the P2P network backbone (signal booster) by
simply being connected to the MQTT broker with high power.
"""
import asyncio
import traceback
from drone.core.comms import MqttClient

class SatelliteRelay:
    def __init__(self, mqtt: MqttClient):
        self.mqtt = mqtt
        # Topics to uplink to Tier 3 Global HQ 
        self.uplink_topics = [
            "mission/start", # All mission triggers
            "fleet/event/+", # All major events (target found, needs relief, etc)
            "fleet/state/+", # All state changes
        ]
        self.satcom_topic_prefix = "global_hq/uplink"
        print("[SatRelay] Initialized. Awaiting messages for uplink.")

    async def run(self):
        """Main run loop for the relay."""
        print(f"[SatRelay] Subscribing to high-priority topics for uplink: {self.uplink_topics}")
        
        for topic in self.uplink_topics:
            await self.mqtt.subscribe(topic)

        # Listen for messages from drones and GCS
        async for topic, payload in self.mqtt.listen():
            try:
                # Check if this topic is one we should uplink
                # (Simple check, can be made more robust)
                is_uplinkable = any(
                    topic.startswith(t.replace("/+", "")) 
                    for t in self.uplink_topics
                )
                
                if is_uplinkable:
                    # Simulate "uplinking" by re-publishing to a new topic
                    # [cite: 35, 36]
                    uplink_topic = f"{self.satcom_topic_prefix}/{topic}"
                    print(f"[SatRelay] Uplinking message from '{topic}' to '{uplink_topic}'")
                    
                    await self.mqtt.publish(
                        uplink_topic,
                        payload,
                        retain=False
                    )
            except Exception as e:
                print(f"[SatRelay] Error handling MQTT message on {topic}: {e}")
                traceback.print_exc()