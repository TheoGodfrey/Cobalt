"""
GCS Server
Runs a WebSocket server for the Ground Control Station (Web UI)
to connect to.
"""

import asyncio
import json
from time import time
import websockets
from typing import Set, Optional
from .fleet_coordinator import FleetCoordinator

class GcsServer:
    """
    Handles WebSocket connections from GCS clients.
    - Receives commands from GCS (e.g., "LAUNCH_MISSION")
    - Pushes fleet state to GCS
    """
    
    def __init__(self, fleet_coord: FleetCoordinator, host: str = "0.0.0.0", port: int = 8765):
        self.fleet_coord = fleet_coord
        self.host = host
        self.port = port
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        print(f"[GcsServer] Initialized. Will listen on {host}:{port}")

    async def _register(self, websocket: websockets.WebSocketServerProtocol):
        """Register a new GCS client."""
        self.connected_clients.add(websocket)
        print(f"[GcsServer] GCS client connected: {websocket.remote_address}")

    async def _unregister(self, websocket: websockets.WebSocketServerProtocol):
        """Unregister a GCS client."""
        self.connected_clients.remove(websocket)
        print(f"[GcsServer] GCS client disconnected: {websocket.remote_address}")

    async def _handle_message(self, websocket: websockets.WebSocketServerProtocol, message_str: str):
        """Process incoming messages from a GCS client."""
        try:
            message = json.loads(message_str)
            command_type = message.get("type")
            
            print(f"[GcsServer] Received command: {command_type}")
            
            if command_type == "LAUNCH_MISSION":
                mission_file = message.get("mission_file")
                if mission_file:
                    # Run as a task so we don't block the server
                    asyncio.create_task(
                        self.fleet_coord.load_and_start_mission(mission_file)
                    )
                else:
                    await websocket.send(json.dumps({"type": "ERROR", "message": "No mission_file provided"}))
                    
            elif command_type == "PING":
                await websocket.send(json.dumps({"type": "PONG", "timestamp": time.monotonic()}))
            
            else:
                await websocket.send(json.dumps({"type": "ERROR", "message": f"Unknown command {command_type}"}))

        except json.JSONDecodeError:
            print("[GcsServer] Received invalid JSON message")
            await websocket.send(json.dumps({"type": "ERROR", "message": "Invalid JSON"}))
        except Exception as e:
            print(f"[GcsServer] Error handling message: {e}")

    async def _connection_handler(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Handle a single client connection."""
        await self._register(websocket)
        try:
            async for message in websocket:
                await self._handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # FIX: Added 'self.' to correct NameError
            await self._unregister(websocket)

    async def broadcast_state(self):
        """Periodically broadcasts the fleet state to all connected GCS clients."""
        while True:
            await asyncio.sleep(1.0) # Broadcast every 1 second
            
            if not self.connected_clients:
                continue
                
            try:
                snapshot = await self.fleet_coord.get_fleet_snapshot()
                
                # FIX: Removed default=str.
                # This relies on get_fleet_snapshot() (Bug #10) 
                # correctly returning a JSON-serializable dict.
                state_message = json.dumps({
                    "type": "FLEET_STATE_UPDATE",
                    "data": snapshot
                })
                
                # Broadcast to all
                await asyncio.wait([
                    client.send(state_message) for client in self.connected_clients
                ])
            except Exception as e:
                print(f"[GGcsServer] Error broadcasting state: {e}")

    async def run(self):
        """Starts the WebSocket server."""
        print(f"[GcsServer] Starting server on ws://{self.host}:{self.port}")
        
        # Start the periodic state broadcaster
        asyncio.create_task(self.broadcast_state())
        
        # Start the WebSocket server
        server = await websockets.serve(
            self._connection_handler,
            self.host,
            self.port
        )
        await server.wait_closed()