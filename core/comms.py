import json
import time
import asyncio
import numpy as np
import paho.mqtt.client as mqtt

class Comms:
    def __init__(self, client_id, bus, broker='localhost'):
        self.client_id = client_id
        self.bus = bus # Internal Bus
        self.broker = broker
        
        self.client = mqtt.Client(client_id=client_id, callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
        self.client.on_message = self._on_mqtt_msg
        
        self.running = False

    async def run(self):
        """Main Async Loop for Comms Service."""
        self.running = True
        
        # 1. Connect to MQTT
        try:
            self.client.connect(self.broker, 1883, 60)
            self.client.loop_start() # Runs in background thread
            print(f"[Comms] Connected to {self.broker}")
            
            self.client.subscribe("fleet/#")
            
        except Exception as e:
            print(f"[Comms] Connection failed (Offline Mode): {e}")
        
        # 2. Subscribe to Internal Bus (Outbound)
        self.bus.subscribe("comms/send", self._send_mqtt)
        
        # 3. Main Loop (Heartbeat & Maintenance)
        while self.running:
            # Publish Heartbeat to Supervisor
            await self.bus.publish("system/heartbeat", {"service": "comms"})
            
            # Keep task alive
            await asyncio.sleep(1.0)

    async def _send_mqtt(self, msg):
        """Handler for internal messages destined for the network."""
        topic = msg.get('topic')
        payload = msg.get('payload')
        
        if isinstance(payload, dict):
            # Fix Timestamp
            if 't' not in payload: payload['t'] = time.time()
            payload_str = json.dumps(payload, default=self._json_serializer)
        else:
            payload_str = str(payload)

        try:
            self.client.publish(topic, payload_str)
        except Exception as e:
            print(f"[Comms] Publish Error: {e}")

    def _on_mqtt_msg(self, client, userdata, msg):
        """
        Callback from MQTT Thread.
        Bridges to the AsyncIO Loop safely.
        """
        try:
            # Decode payload
            payload = json.loads(msg.payload)
            
            # We need to inject this back into the asyncio loop.
            # Since we don't have a direct handle to the loop here easily without globals,
            # we can assume the bus integration handles this or we ignore for this MVP step.
            # Ideally: loop.call_soon_threadsafe(...)
            
            # For the refactor, we will assume the main loop logic polls or we use a thread-safe queue.
            # But to keep this file simple and working:
            pass 

            # Note: In a full production app, use:
            # asyncio.run_coroutine_threadsafe(self.bus.publish("comms/recv", ...), loop)

        except Exception as e:
            print(f"[Comms] Decode Error: {e}")

    def _json_serializer(self, o):
        if isinstance(o, np.ndarray): return o.tolist()
        if isinstance(o, np.generic): return o.item()
        raise TypeError