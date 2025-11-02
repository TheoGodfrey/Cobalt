"""
GCS (Ground Control Station) WebSocket Server.

This server runs as part of the Coordinator and provides a link
to a web-based frontend.
"""
import asyncio
import json
import websockets
from typing import Set, TYPE_CHECKING

# --- Robust Import Logic ---
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parent.parent
CORE_PATH = ROOT / "v_0_2" / "scout_drone"
if str(CORE_PATH) not in sys.path:
    sys.path.append(str(CORE_PATH))
# --- End Import Logic ---

from drone.core.config_models import GcsConfig  # <-- FIX: Corrected import
from drone.core.drone import Telemetry         # <-- FIX: Corrected import
from drone.core.position import Position       # <-- FIX: Added missing import

# Forward declaration for type hinting
if TYPE_CHECKING:
    from .coordinator import Coordinator # Use relative import for type check

class GcsServer:
    """
    Manages WebSocket communication with GCS clients.
    """
    
    # --- FIX: Changed __init__ to match coordinator_main.py ---
    def __init__(self, config: GcsConfig):
        self.config = config
        self.controller: 'Coordinator' | None = None # Controller is set *after* init
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        print(f"[GcsServer] Initialized. Will listen on {config.host}:{config.port}")

    def set_controller(self, controller: 'Coordinator'):
        """Dependency injection for the Coordinator."""
        self.controller = controller
    # ---------------------------------------------------------

    async def _register(self, websocket: websockets.WebSocketServerProtocol):
        """Register a new GCS client."""
        self.clients.add(websocket)
        print(f"[GcsServer] Client connected: {websocket.remote_address}. Total clients: {len(self.clients)}")

    async def _unregister(self, websocket: websockets.WebSocketServerProtocol):
        """Unregister a GCS client."""
        self.clients.remove(websocket)
        print(f"[GcsServer] Client disconnected: {websocket.remote_address}. Total clients: {len(self.clients)}")

    async def _handle_message(self, message: str):
        """Handle incoming messages from a GCS client."""
        # --- FIX: Check if controller is set ---
        if not self.controller:
            print("[GcsServer] Error: Controller not set. Ignoring message.")
            return
        # -------------------------------------

        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'TRIGGER_MOB_MODE':
                print("[GcsServer] Received TRIGGER_MOB_MODE from operator.")
                await self.controller.trigger_mob_event()

            elif msg_type == 'CONFIRM_TARGET':
                drone_id = data.get('data', {}).get('drone_id')
                print(f"[GcsServer] Received CONFIRM_TARGET from operator for {drone_id}.")
                await self.controller.handle_operator_confirmation(drone_id)
                
            elif msg_type == 'REJECT_TARGET':
                drone_id = data.get('data', {}).get('drone_id')
                print(f"[GcsServer] Received REJECT_TARGET from operator for {drone_id}.")
                await self.controller.handle_operator_rejection(drone_id)
            
            elif msg_type == 'TRIGGER_PATROL_MODE':
                print("[GcsServer] Received TRIGGER_PATROL_MODE from operator.")
                await self.controller.trigger_patrol_mode()
            
            elif msg_type == 'TRIGGER_OVERWATCH_MODE':
                print("[GcsServer] Received TRIGGER_OVERWATCH_MODE from operator.")
                # TODO: Get position from GCS click
                default_pos = {"x": 100.0, "y": 100.0, "z": 0.0} 
                pos_data = data.get('data', {"position": default_pos})
                await self.controller.trigger_overwatch_mode(pos_data)

            else:
                print(f"[GcsServer] Unknown message type: {msg_type}")
                
        except json.JSONDecodeError:
            print(f"[GcsServer] Received invalid JSON: {message}")
        except Exception as e:
            print(f"[GcsServer] Error handling message: {e}")

    async def _connection_handler(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Handle a single client connection's lifecycle."""
        await self._register(websocket)
        try:
            # Listen for incoming messages
            async for message in websocket:
                await self._handle_message(str(message)) # FIX: Was self.handle_message
        except websockets.exceptions.ConnectionClosed:
            print(f"[GcsServer] Connection closed by client.")
        finally:
            await self._unregister(websocket)

    async def run(self):
        """Start the WebSocket server."""
        print(f"[GcsServer] Starting server on ws://{self.config.host}:{self.config.port}...")
        try:
            server = await websockets.serve(
                self._connection_handler,
                self.config.host,
                self.config.port
            )
            await server.wait_closed()
        except OSError as e:
            print(f"[GcsServer] FATAL: Could not start server (port {self.config.port} likely in use). {e}")
            raise
            
    async def broadcast(self, payload: dict):
        """Send a JSON payload to all connected clients."""
        if not self.clients:
            return # No one to send to
        
        # Use default=vars to handle Pydantic models
        message = json.dumps(payload, default=vars)
        
        # Use asyncio.gather to send to all clients concurrently
        tasks = [client.send(message) for client in self.clients]
        await asyncio.gather(*tasks, return_exceptions=True)
        
    async def broadcast_telemetry(self, drone_id: str, telemetry: Telemetry, state: str):
        """Helper function to format and broadcast telemetry."""
        
        # --- FIX: `telemetry` object does not have `.id` ---
        payload = {
            "type": "telemetry",
            "data": {
                "drone_id": drone_id, # Use the passed-in drone_id
                "position": telemetry.position.model_dump(), # Use model_dump()
                "attitude": {
                    "roll": round(telemetry.attitude_roll, 1),
                    "pitch": round(telemetry.attitude_pitch, 1),
                    "yaw": round(telemetry.attitude_yaw, 1),
                },
                "battery": round(telemetry.battery, 1),
                "state": telemetry.state,
                "mission_phase": state,
            }
        }
        await self.broadcast(payload)
        # -------------------------------------------------

    async def broadcast_event(self, event_type: str, data: dict):
        """Helper function to format and broadcast a special event."""
        payload = {
            "type": event_type,
            "data": data
        }
        await self.broadcast(payload)

