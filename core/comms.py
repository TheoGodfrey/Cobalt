"""
Communication Layer - Thread-Safe MQTT Bridge
Fixes: Race condition between MQTT callbacks and asyncio event loop
"""
import asyncio
import json
import time
import paho.mqtt.client as mqtt


class Comms:
    """
    Thread-safe MQTT communication bridge.
    
    CRITICAL FIX: MQTT callbacks run in separate thread from asyncio.
    Solution: Use asyncio.run_coroutine_threadsafe() to safely inject
    messages from MQTT thread into asyncio event loop.
    """
    
    def __init__(self, client_id, bus, broker='localhost'):
        self.client_id = client_id
        self.bus = bus
        self.broker = broker
        
        self.client = mqtt.Client(client_id)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_mqtt_msg
        
        self.loop = None  # Set when run() is called
        self.connected = False
        
        print(f"[Comms] Initialized for {client_id}")
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"[Comms] Connected to {self.broker}")
        else:
            print(f"[Comms] Connection failed: {rc}")
    
    def _on_mqtt_msg(self, client, userdata, msg):
        """
        CRITICAL: This runs in MQTT's thread, NOT asyncio's thread.
        Must use run_coroutine_threadsafe() to safely publish to bus.
        """
        try:
            payload = json.loads(msg.payload)
            topic = msg.topic
            
            # Bridge MQTT thread → Asyncio loop safely
            if self.loop and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self._handle_message(topic, payload),
                    self.loop
                )
            else:
                print(f"[Comms] Warning: Event loop not ready, dropping message")
                
        except json.JSONDecodeError:
            print(f"[Comms] Invalid JSON on {msg.topic}")
        except Exception as e:
            print(f"[Comms] Error handling message: {e}")
    
    async def _handle_message(self, topic, payload):
        """
        Async handler (runs in asyncio thread).
        Publishes to internal message bus.
        """
        # Map MQTT topics to internal bus events
        if topic.startswith("fleet/telemetry"):
            await self.bus.publish("fleet/telemetry", payload)
        elif topic.startswith("fleet/home"):
            await self.bus.publish("fleet/home", payload)
        elif topic.startswith("fleet/detection"):
            await self.bus.publish("fleet/detection", payload)
        else:
            # Generic passthrough
            await self.bus.publish(f"mqtt/{topic}", payload)
    
    async def run(self):
        """
        Async run loop. Must be called from asyncio context.
        Captures event loop reference for thread-safe callbacks.
        """
        # CRITICAL: Capture loop reference before starting MQTT
        self.loop = asyncio.get_running_loop()
        
        try:
            self.client.connect(self.broker, 1883, 60)
        except Exception as e:
            print(f"[Comms] Failed to connect: {e}")
            print(f"[Comms] Running in OFFLINE mode")
            return
        
        # Subscribe to fleet topics
        self.client.subscribe("fleet/#")
        
        # Start MQTT network loop in background thread
        self.client.loop_start()
        
        print(f"[Comms] Running (loop captured: {id(self.loop)})")
        
        # Keep service alive
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            print("[Comms] Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()
    
    def publish(self, topic, payload):
        """
        Synchronous publish (safe to call from any thread).
        """
        if self.connected:
            self.client.publish(topic, json.dumps(payload))
        else:
            print(f"[Comms] Not connected, cannot publish to {topic}")
    
    async def publish_async(self, topic, payload):
        """
        Async publish wrapper (for use in async contexts).
        """
        self.publish(topic, payload)
        await asyncio.sleep(0)  # Yield to event loop


# Example integration with message bus
class MessageBus:
    """
    Internal message bus for inter-service communication.
    All services publish/subscribe through this.
    """
    def __init__(self):
        self.subscribers = {}
    
    def subscribe(self, topic, callback):
        """Subscribe to topic with async callback"""
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)
    
    async def publish(self, topic, data):
        """Publish data to all subscribers of topic"""
        if topic in self.subscribers:
            for callback in self.subscribers[topic]:
                try:
                    await callback(data)
                except Exception as e:
                    print(f"[Bus] Error in subscriber: {e}")