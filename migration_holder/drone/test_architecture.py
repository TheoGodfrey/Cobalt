"""
Test that the v_0.2 architecture is sound and components are importable.
This module verifies that:
- Core data classes (Position) are independent.
- Core object classes (Drone) can be instantiated.
- The main configuration file (mission_config.yaml) is valid.
- All strategy factories can create their respective strategies.
- Drone health tracking logic works.
"""
import sys
import yaml
import asyncio
from pathlib import Path
from pydantic import ValidationError

# --- Add parent directory to path ---
# This allows 'core' and 'strategies' to be imported
FILE = Path(__file__).resolve()
ROOT = FILE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
# ------------------------------------

try:
    # Core components
    from core.position import Position
    from core.drone import Drone, SimulatedFlightController
    from core.config_models import (
        Settings, LawnmowerConfig, OrbitConfig, 
        PrecisionHoverConfig, VerticalAscentConfig
    )
    
    # Strategy factories
    from strategies import get_search_strategy, get_flight_strategy
    from strategies.search.random import RandomSearchStrategy
    from strategies.search.vertical_ascent import VerticalAscentSearchStrategy
    from strategies.search.lawnmower import LawnmowerSearchStrategy
    from strategies.flight.direct import DirectFlightStrategy
    from strategies.flight.precision_hover import PrecisionHoverFlightStrategy
    from strategies.flight.orbit import OrbitFlightStrategy

except ImportError as e:
    print(f"FATAL: Failed to import necessary modules: {e}")
    print("Please ensure you are running this test from the 'v_0.2/scout_drone' directory")
    print(f"Sys path: {sys.path}")
    sys.exit(1)


def test_position_is_standalone():
    """Verify Position can be used independently"""
    print("Testing: Position independence...")
    pos1 = Position(x=100, y=200, z=15)
    pos2 = Position(x=150, y=250, z=15)
    distance = pos1.distance_to(pos2)
    assert distance > 0
    assert "x=100.0" in str(pos1)
    print("✓ Position works independently")

def test_can_create_multiple_drones():
    """Verify drones have unique IDs and use the correct constructor"""
    print("Testing: Multiple drone creation...")
    # Drones now require a controller object
    drone1 = Drone(SimulatedFlightController(), drone_id="drone_1")
    drone2 = Drone(SimulatedFlightController(), drone_id="drone_2")
    assert drone1.id != drone2.id
    assert drone1.id == "drone_1"
    assert drone2.id == "drone_2"
    print("✓ Multiple drones can be created with unique IDs")

def test_config_loading():
    """Verify the main config/mission_config.yaml is valid"""
    print("Testing: Main config file validation...")
    config_path = FILE.parent / "config" / "mission_config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}")
        
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # This will raise a ValidationError if anything is wrong
    settings = Settings(**config_data)
    
    assert settings.mission.max_search_iterations > 0
    assert settings.cameras.thermal.type == 'simulated'
    assert settings.cameras.visual.intrinsics.width == 640
    assert settings.strategies.search.algorithm == 'vertical_ascent'
    print("✓ Main config file loaded and validated successfully")

def test_strategy_factories():
    """Verify all strategy factories can create their strategies"""
    print("Testing: Strategy factory instantiation...")
    
    # 1. Create mock config objects
    lawn_cfg = LawnmowerConfig(patrol_altitude=50, spacing=20, leg_length=100, num_legs=5)
    orbit_cfg = OrbitConfig(radius=30, speed=5, altitude_offset=10)
    hover_cfg = PrecisionHoverConfig(altitude_offset=3.0)
    ascent_cfg = VerticalAscentConfig(max_altitude=100, step_size=10)
    
    # 2. Test factories
    s_random = get_search_strategy('random', None)
    s_ascent = get_search_strategy('vertical_ascent', ascent_cfg)
    s_lawn = get_search_strategy('lawnmower', lawn_cfg)
    
    f_direct = get_flight_strategy('direct', None)
    f_hover = get_flight_strategy('precision_hover', hover_cfg)
    f_orbit = get_flight_strategy('orbit', orbit_cfg)
    
    # 3. Assert types
    assert isinstance(s_random, RandomSearchStrategy)
    assert isinstance(s_ascent, VerticalAscentSearchStrategy)
    assert isinstance(s_lawn, LawnmowerSearchStrategy)
    assert isinstance(f_direct, DirectFlightStrategy)
    assert isinstance(f_hover, PrecisionHoverFlightStrategy)
    assert isinstance(f_orbit, OrbitFlightStrategy)
    
    # 4. Assert config values were passed
    assert s_ascent.max_altitude == 100
    assert s_lawn.config.spacing == 20
    assert f_hover.hover_altitude_offset == 3.0
    assert f_orbit.config.radius == 30
    
    print("✓ All strategy factories work correctly")

async def test_health_tracking():
    """Verify health tracking works with async telemetry"""
    print("Testing: Health tracking...")
    drone = Drone(SimulatedFlightController(), drone_id="test_drone")
    
    # Connect drone to initialize telemetry
    await drone.connect()
    
    assert len(drone.health_history) == 0
    
    await drone.update_telemetry()
    assert len(drone.health_history) == 1
    
    for _ in range(15):
        await drone.update_telemetry()
    
    # Should keep only last 10
    assert len(drone.health_history) == 10
    
    # Check that data is being recorded
    assert drone.health_history[-1].battery < 100.0
    print("✓ Health tracking works correctly")


def run_async_test(test_func):
    """Simple helper to run a single async test"""
    try:
        asyncio.run(test_func())
        return True
    except AssertionError as e:
        print(f"✗ {test_func.__name__} failed: {e}")
    except Exception as e:
        print(f"✗ {test_func.__name__} error: {e}")
    return False

def run_sync_test(test_func):
    """Simple helper to run a single sync test"""
    try:
        test_func()
        return True
    except AssertionError as e:
        print(f"✗ {test_func.__name__} failed: {e}")
    except Exception as e:
        print(f"✗ {test_func.__name__} error: {e}")
    return False


def run_all_tests():
    """Run all architecture validation tests"""
    print("\n" + "="*50)
    print("Architecture Validation Tests (v_0.2)")
    print("="*50 + "\n")
    
    # List of all test functions (sync and async)
    sync_tests = [
        test_position_is_standalone,
        test_can_create_multiple_drones,
        test_config_loading,
        test_strategy_factories
    ]
    async_tests = [
        test_health_tracking
    ]
    
    passed = 0
    failed = 0
    
    for test in sync_tests:
        if run_sync_test(test):
            passed += 1
        else:
            failed += 1
        print() # Newline for readability
    
    for test in async_tests:
        if run_async_test(test):
            passed += 1
        else:
            failed += 1
        print() # Newline for readability

    print("="*50)
    print(f"Results: {passed} passed, {failed} failed")
    print("="*50)
    
    if failed == 0:
        print("\n✅ All tests passed! Architecture is sound.\n")
    else:
        print(f"\n❌ {failed} test(s) failed. Please fix before proceeding.\n")
    
    return failed == 0

if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)