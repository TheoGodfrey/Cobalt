# core/supervisor.py
import asyncio
import time
import traceback

class ServiceSupervisor:
    """
    Manages the lifecycle of async tasks (Services).
    Restarts them if they crash or fail to heartbeat.
    """
    def __init__(self, bus):
        self.bus = bus
        self.services = {} # {name: coroutine_func}
        self.tasks = {}    # {name: asyncio.Task}
        self.heartbeats = {} # {name: timestamp}
        self.max_silence = 5.0 # Seconds before restart

    def register(self, name, service_func):
        self.services[name] = service_func
        self.heartbeats[name] = time.time()

    async def start(self):
        """Start all services and the watchdog."""
        print("[Supervisor] Starting Services...")
        
        # 1. Launch all registered services
        for name, func in self.services.items():
            self._launch_service(name)

        # 2. Subscribe to heartbeats
        self.bus.subscribe("system/heartbeat", self._on_heartbeat)

        # 3. Start Watchdog Loop
        await self._watchdog_loop()

    def _launch_service(self, name):
        print(f"[Supervisor] Launching {name}...")
        # Wrap the service to catch crashes
        self.tasks[name] = asyncio.create_task(self._safe_runner(name))
        self.heartbeats[name] = time.time() # Reset heartbeat on launch

    async def _safe_runner(self, name):
        try:
            await self.services[name]()
        except asyncio.CancelledError:
            print(f"[Supervisor] {name} stopped (Cancelled).")
        except Exception as e:
            print(f"[Supervisor] CRITICAL: {name} CRASHED: {e}")
            traceback.print_exc()
            # The watchdog will catch the silence and restart it

    async def _on_heartbeat(self, msg):
        service_name = msg.get('service')
        if service_name:
            self.heartbeats[service_name] = time.time()

    async def _watchdog_loop(self):
        while True:
            now = time.time()
            for name in list(self.services.keys()):
                last_beat = self.heartbeats.get(name, 0)
                
                if now - last_beat > self.max_silence:
                    print(f"[Supervisor] WATCHDOG: {name} is silent ({now - last_beat:.1f}s). Restarting...")
                    
                    # 1. Kill old task
                    if name in self.tasks:
                        self.tasks[name].cancel()
                        try:
                            await self.tasks[name]
                        except asyncio.CancelledError:
                            pass
                    
                    # 2. Restart
                    self._launch_service(name)
            
            await asyncio.sleep(1.0)