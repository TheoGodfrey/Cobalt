import json
import time
import asyncio
import paho.mqtt.client as mqtt
import numpy as np

class Comms:
    def __init__(self, client_id, broker='localhost', neighbor_ttl=2.0):
        self.client_id = client_id
        self.client = mqtt.Client(client_id=client_id)
        self.client.on_message = self._on_msg
        self.broker = broker
        self.callbacks = {}
        
        # Neighbor Tracking (Decentralized)
        self.neighbors = {} # {drone_id: {'pos': np.array, 'last_seen': float}}
        self.neighbor_ttl = neighbor_ttl # Seconds before pruning

    def connect(self):
        try:
            self.client.connect(self.broker, 1883, 60)
            self.client.loop_start()
            print(f"[Comms] Connected to {self.broker}")
            
            # Automatically subscribe to fleet telemetry for decentralized awareness
            self.sub("fleet/telemetry", self._handle_neighbor_telemetry)
            
            # Start background pruning task
            asyncio.create_task(self._prune_neighbors_loop())
            
        except Exception as e:
            print(f"[Comms] Offline Mode (Error: {e})")

    def sub(self, topic, cb):
        self.client.subscribe(topic)
        self.callbacks[topic] = cb

    def pub(self, topic, data):
        try:
            # Handle numpy serialization
            def default(o):
                if isinstance(o, np.ndarray): return o.tolist()
                if isinstance(o, np.generic): return o.item()
                raise TypeError
                
            self.client.publish(topic, json.dumps(data, default=default))
        except Exception as e:
            print(f"[Comms] Publish failed: {e}")

    def _on_msg(self, c, u, msg):
        if msg.topic in self.callbacks:
            try:
                payload = json.loads(msg.payload)
                # If callback is async, schedule it? For now assume sync/fast
                self.callbacks[msg.topic](payload)
            except Exception as e:
                print(f"[Comms] Msg Error: {e}")

    def _handle_neighbor_telemetry(self, msg):
        """
        Automatic handler for neighbor updates.
        """
        sender_id = msg.get('id')
        if sender_id and sender_id != self.client_id:
            self.neighbors[sender_id] = {
                'pos': np.array(msg['pos']),
                'vel': np.array(msg.get('vel', [0,0,0])),
                'last_seen': time.time()
            }

    async def _prune_neighbors_loop(self):
        """
        Periodically removes neighbors we haven't heard from.
        """
        while True:
            now = time.time()
            # Create list of dead neighbors
            dead = [nid for nid, data in self.neighbors.items() 
                    if now - data['last_seen'] > self.neighbor_ttl]
            
            for nid in dead:
                del self.neighbors[nid]
                # print(f"[Comms] Neighbor {nid} timed out (pruned).")
                
            await asyncio.sleep(1.0)

    def get_neighbors(self):
        """Returns simple dict of {id: pos} for Equations to use."""
        return {nid: data['pos'] for nid, data in self.neighbors.items()}