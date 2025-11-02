"""
MQTT Communication Bus for the Drone-MOB fleet.

Handles connection, asynchronous publishing, and asynchronous message subscription.
"""
import asyncio
import json
import paho.mqtt.client as mqtt
from .config_models import MqttConfig
from typing import AsyncGenerator, Tuple

class MqttClient:
    """Async wrapper for the Paho MQTT client."""
    
    def __init__(self, config: MqttConfig, client_id: str):
        self.config = config
        self.client_id = client_id
        
        # Paho client setup
        self._client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        
        # Async queue for decoupling Paho's thread from asyncio
        self.incoming_messages: asyncio.Queue = asyncio.Queue()
        self.is_connected = False

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """Paho callback for when connection is established."""
        if reason_code == 0:
            print(f"[{self.client_id} MQTT] Connected to broker at {self.config.host}")
            self.is_connected = True
            # Resubscribe to topics if needed (handled by subscribe method)
        else:
            print(f"[{self.client_id} MQTT] Failed to connect: {reason_code}")
            self.is_connected = False



    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        """Paho callback for when disconnected."""
        print(f"[{self.client_id} MQTT] Disconnected with reason: {reason_code}")
        self.is_connected = False

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage):
        """Paho callback for all subscribed messages."""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode('utf-8'))
            # Put the parsed message into the async queue
            self.incoming_messages.put_nowait((topic, payload))
        except json.JSONDecodeError:
            print(f"[{self.client_id} MQTT] Received non-JSON message on {msg.topic}")
        except Exception as e:
            print(f"[{self.client_id} MQTT] Error in on_message: {e}")

    async def connect(self, will_topic: str | None = None, will_payload: str | None = None): 
        """Asynchronously connect to the MQTT broker, optionally setting LWT."""
        print(f"[{self.client_id} MQTT] Attempting connection to {self.config.host}...")
        try:
            
            # --- NEW LWT LOGIC for Hub Resilience ---
            if will_topic and will_payload:
                print(f"[{self.client_id} MQTT] Setting Last Will: {will_topic}={will_payload}")
                self._client.will_set(
                    f"{self.config.base_topic}/{will_topic}",
                    will_payload,
                    qos=1,
                    retain=True
                )
            # ----------------------------------------
            
            self._client.connect(self.config.host, self.config.port, 60)
            self._client.loop_start() # Starts Paho's network thread
            
            # Wait for the connection to be established
            for _ in range(10):
                if self.is_connected:
                    return
                await asyncio.sleep(0.5)
            
            print(f"[{self.client_id} MQTT] Connection timed out.")
            self._client.loop_stop()

        except Exception as e:
            print(f"[{self.client_id} MQTT] Connection error: {e}")

    async def disconnect(self):
        """Disconnect from the broker."""
        print(f"[{self.client_id} MQTT] Disconnecting...")
        self._client.loop_stop() # Stop the network thread
        self._client.disconnect()

    async def publish(self, topic: str, payload: dict, retain: bool = False):
        """Publish an asynchronous JSON message."""
        if not self.is_connected:
            print(f"[{self.client_id} MQTT] Not connected. Cannot publish to {topic}")
            return
            
        full_topic = f"{self.config.base_topic}/{topic}"
        message = json.dumps(payload)
        self._client.publish(full_topic, message, qos=1, retain=retain)

    async def subscribe(self, topic: str):
        """Subscribe to a topic."""
        if not self.is_connected:
            print(f"[{self.client_id} MQTT] Not connected. Cannot subscribe.")
            return

        full_topic = f"{self.config.base_topic}/{topic}"
        print(f"[{self.client_id} MQTT] Subscribing to {full_topic}")
        self._client.subscribe(full_topic, qos=1)

    async def listen(self) -> AsyncGenerator[Tuple[str, dict], None]:
        """Async generator to yield messages from the queue."""
        while True:
            topic, payload = await self.incoming_messages.get()
            # Strip base topic to make it easier to handle
            short_topic = topic.removeprefix(f"{self.config.base_topic}/")
            yield short_topic, payload

