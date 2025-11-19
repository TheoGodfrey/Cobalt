# core/bus.py
import asyncio
from collections import defaultdict
from typing import Callable, Any, Awaitable

class MessageBus:
    """
    Local Async Pub/Sub Bus.
    Decouples architecture: Comms <-> Bus <-> Logic
    """
    def __init__(self):
        self.subscribers = defaultdict(list)

    def subscribe(self, topic: str, callback: Callable[[Any], Awaitable[None]]):
        """Register an async callback for a topic."""
        self.subscribers[topic].append(callback)

    async def publish(self, topic: str, data: Any):
        """Publish data to a topic. Fire-and-forget."""
        if topic in self.subscribers:
            # Run all callbacks concurrently
            await asyncio.gather(
                *[cb(data) for cb in self.subscribers[topic]], 
                return_exceptions=True
            )