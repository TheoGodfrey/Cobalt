import json
import paho.mqtt.client as mqtt

class Comms:
    def __init__(self, client_id, broker='localhost'):
        self.client = mqtt.Client(client_id=client_id)
        self.client.on_message = self._on_msg
        self.broker = broker
        self.callbacks = {}

    def connect(self):
        try:
            self.client.connect(self.broker, 1883, 60)
            self.client.loop_start()
            print(f"[Comms] Connected to {self.broker}")
        except:
            print("[Comms] Offline Mode")

    def sub(self, topic, cb):
        self.client.subscribe(topic)
        self.callbacks[topic] = cb

    def pub(self, topic, data):
        self.client.publish(topic, json.dumps(data))

    def _on_msg(self, c, u, msg):
        if msg.topic in self.callbacks:
            self.callbacks[msg.topic](json.loads(msg.payload))