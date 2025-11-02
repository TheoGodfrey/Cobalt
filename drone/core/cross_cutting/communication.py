"""
Component 11: Communication Layer
Provides an asynchronous MQTT client for communication between the
drone, the hub, and the GCS.

This implementation uses paho-mqtt's asyncio support.
"""

import asyncio
import json
import logging
from typing import Callable, Any, Dict, Optional
import paho.mqtt.client as mqtt

# Configure logging
log = logging.getLogger(__name__)

# Type alias for the async message callback
# The callback will receive the topic and the deserialized JSON payload
AsyncMsgCallback = Callable[[str, Dict[str, Any]], asyncio.Future]
LwtCallback = Callable[[str], asyncio.Future]

class MqttClient:
    """
    An asynchronous MQTT client wrapper.
    """
    
    # Topic definitions
    LWT_TOPIC_PREFIX = "fleet/lwt"
    
    def __init__(self, client_id: str, host: str = "localhost", port: int = 1883, user: Optional[str] = None, password: Optional[str] = None, config: Optional[Dict] = None):
        """
        Initializes the MQTT client.
        
        Args:
            client_id: The unique ID for this client.
            host: The MQTT broker hostname.
            port: The MQTT broker port.
            user: The MQTT username (optional).
            password: The MQTT password (optional).
            config: Optional config dict (e.g., from drone/main.py).
        """
        self._client_id = client_id
        self._broker_host = config.get("host", host) if config else host
        self._broker_port = config.get("port", port) if config else port
        
        # Paho client setup
        self._client = mqtt.Client(client_id=client_id)
        self._client.on_connect = self._on_connect

        # --- FIX: Set username and password ---
        if user and password:
            self._client.username_pw_set(user, password)
        # --- End of FIX ---

        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        
        # Use asyncio for the event loop
        self._client.loop_start() # Starts a background thread for network I/O
        
        self._loop = asyncio.get_running_loop()
        # Subscription management
        # topic -> async callback
        self._subscriptions: Dict[str, AsyncMsgCallback] = {}
        self._lwt_callback: Optional[LwtCallback] = None
        
        log.info(f"[MQTT-{self._client_id}] Initialized. Broker: {self._broker_host}:{self._broker_port}")

    def _on_connect(self, client, userdata, flags, rc):
        """Paho callback for connection."""
        if rc == 0:
            log.info(f"[MQTT-{self._client_id}] Connected to broker.")
            # Re-subscribe to all topics on reconnection
            for topic in self._subscriptions:
                self._client.subscribe(topic)
                log.debug(f"[MQTT-{self._client_id}] Re-subscribed to {topic}")
            
            # Subscribe to LWT topic if a callback is registered
            if self._lwt_callback:
                lwt_topic = f"{self.LWT_TOPIC_PREFIX}/+"
                self._client.subscribe(lwt_topic)
                log.debug(f"[MQTT-{self._client_id}] Re-subscribed to LWT topic {lwt_topic}")
        else:
            log.error(f"[MQTT-{self._client_id}] Connection failed with code {rc}")

    def _on_disconnect(self, client, userdata, rc):
        """Paho callback for disconnection."""
        log.warning(f"[MQTT-{self._client_id}] Disconnected from broker (rc: {rc}). Reconnecting...")
        # Paho's loop_start() handles automatic reconnection

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage):
        """Paho callback for incoming messages."""
        topic = msg.topic
        payload_str = msg.payload.decode('utf-8')
        
        # --- Handle LWT Messages ---
        # This fixes Bug #13 by handling messages on the LWT topic
        if topic.startswith(self.LWT_TOPIC_PREFIX) and self._lwt_callback:
            try:
                # LWT message payload is just the drone_id or status
                # Here we assume the topic itself contains the ID
                drone_id = topic.split('/')[-1]
                data = json.loads(payload_str)
                
                if data.get("status") == "offline": 
                    # Pass the disconnected drone_id to the callback
                    # --- FIX: Use threadsafe call to the main loop ---
                    coro = self._lwt_callback(drone_id)
                    asyncio.run_coroutine_threadsafe(coro, self._loop)
                
            except Exception as e:
                log.error(f"[MQTT-{self._client_id}] Error processing LWT message on {topic}: {e}")
            return # Stop processing

        # --- Handle Standard Subscriptions ---
        callback = self._subscriptions.get(topic)
        
        # Check for wildcard subscriptions
        if not callback:
            for sub_topic, cb in self._subscriptions.items():
                if mqtt.topic_matches_sub(sub_topic, topic):
                    callback = cb
                    break
        
        if callback:
            try:
                # Deserialize JSON payload
                payload_dict = json.loads(payload_str)
                # Schedule the async callback to run
                # --- FIX: Use threadsafe call to the main loop ---
                coro = callback(topic, payload_dict)
                asyncio.run_coroutine_threadsafe(coro, self._loop)
                # --- End of FIX ---
            except json.JSONDecodeError:
                log.warning(f"[MQTT-{self._client_id}] Received non-JSON message on {topic}")
            except Exception as e:
                log.error(f"[MQTT-{self._client_id}] Error in message callback for {topic}: {e}")
        else:
            log.debug(f"[MQTT-{self._client_id}] Received message on unhandled topic: {topic}")

    async def connect(self):
        """
        Asynchronously connects to the MQTT broker.
        Sets the Last Will and Testament (LWT) for this client.
        """
        try:
            # Set LWT: If this client disconnects uncleanly,
            # publish an 'offline' message to its LWT topic.
            lwt_payload = json.dumps({"status": "offline", "drone_id": self._client_id})
            self._client.will_set(
                f"{self.LWT_TOPIC_PREFIX}/{self._client_id}",
                payload=lwt_payload,
                qos=1,
                retain=True
            )
            
            # Connect to the broker
            self._client.connect_async(self._broker_host, self._broker_port, 60)
            
            # Wait for connection to be established
            while not self._client.is_connected():
                await asyncio.sleep(0.1)
                
            # Publish an 'online' message to our LWT topic
        # Publish an 'online' message to our LWT topic
            online_payload = {"status": "online", "drone_id": self._client_id}
            await self.publish(f"{self.LWT_TOPIC_PREFIX}/{self._client_id}", online_payload, retain=True)
        
        except Exception as e:
            log.error(f"[MQTT-{self._client_id}] Failed to connect: {e}")

    async def disconnect(self):
        """Disconnects the client gracefully."""
        log.info(f"[MQTT-{self._client_id}] Disconnecting...")
        # Publish 'offline' message gracefully
        lwt_payload = {"status": "offline", "drone_id": self._client_id}
        await self.publish(f"{self.LWT_TOPIC_PREFIX}/{self._client_id}", lwt_payload, retain=True)
        
        self._client.loop_stop() # Stop the background thread
        self._client.disconnect()

    async def publish(self, topic: str, payload: Dict[str, Any], retain: bool = False, qos: int = 0):
        """
        Asynchronously publishes a JSON message to a topic.
        
        Args:
            topic: The MQTT topic.
            payload: A dictionary to be serialized to JSON.
            retain: Whether to retain the message.
            qos: Quality of Service level (0, 1, or 2).
        """
        try:
            payload_str = json.dumps(payload, default=str)
            msg_info = self._client.publish(topic, payload_str, qos=qos, retain=retain)
            
            # We can wait for publish to complete if QoS > 0
            if qos > 0:
                while not msg_info.is_published():
                    await asyncio.sleep(0.01)
                    
        except Exception as e:
            log.error(f"[MQTT-{self._client_id}] Failed to publish to {topic}: {e}")

    async def subscribe(self, topic: str, callback: AsyncMsgCallback):
        """
        Subscribes to a topic and registers an async callback.
        
        Args:
            topic: The topic (can include wildcards like '+').
            callback: The async function to call on message.
        """
        if topic in self._subscriptions:
            log.warning(f"[MQTT-{self._client_id}] Topic {topic} already has a subscription. Overwriting.")
            
        self._subscriptions[topic] = callback
        
        if self._client.is_connected():
            self._client.subscribe(topic)
            
        log.info(f"[MQTT-{self._client_id}] Subscribed to {topic}")

    async def subscribe_lwt(self, callback: LwtCallback):
        """
        FIX for Bug #13.
        Subscribes to the fleet's LWT topic to listen for disconnections.
        
        Args:
            callback: An async function that accepts a single arg (drone_id).
        """
        lwt_topic = f"{self.LWT_TOPIC_PREFIX}/+"
        self._lwt_callback = callback
        
        if self._client.is_connected():
            self._client.subscribe(lwt_topic)
            
        log.info(f"[MQTT-{self._client_id}] Subscribed to LWT topic: {lwt_topic}")

    async def run(self):
        """
        A placeholder run loop to keep the client alive.
        Paho's `loop_start()` handles the actual work in a thread.
        This async task just keeps the asyncio event loop aware.
        """
        while True:
            await asyncio.sleep(10)
            if not self._client.is_connected():
                log.warning(f"[MQTT-{self._client_id}] Run loop: Client is disconnected.")