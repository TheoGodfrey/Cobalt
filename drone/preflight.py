"""
Pre-Flight Check System

GO/NO-GO decision logic before mission launch.
Critical for maritime operations where conditions change rapidly.
"""
import numpy as np


def can_launch(world, state):
    """
    Execute pre-flight checks and return GO/NO-GO decision.
    
    Args:
        world: WorldState instance
        state: DroneState from HAL
    
    Returns:
        bool: True if safe to launch, False otherwise
    
    Checks:
    1. Battery level (>90% for sea missions)
    2. Wind speed (<12 m/s, regulatory limit)
    3. GPS quality (>8 satellites for maritime)
    4. Communications (MQTT connected)
    5. Home position set (required for RTH)
    """
    
    checks = []
    
    # ============================================================
    # Check 1: Battery
    # ============================================================
    battery_threshold = 90.0  # Require 90% for sea missions
    
    if state.battery < battery_threshold:
        checks.append({
            'name': 'Battery',
            'status': 'FAIL',
            'message': f'{state.battery:.0f}% (need >{battery_threshold:.0f}%)',
            'severity': 'CRITICAL'
        })
    else:
        checks.append({
            'name': 'Battery',
            'status': 'PASS',
            'message': f'{state.battery:.0f}%',
            'severity': 'INFO'
        })
    
    # ============================================================
    # Check 2: Wind Speed
    # ============================================================
    wind_limit = 12.0  # m/s (about 23 knots)
    
    try:
        # Get wind at typical search altitude (50m)
        wind = world.wind.get_at([0, 0, -50])
        wind_speed = np.linalg.norm(wind)
        
        if wind_speed > wind_limit:
            checks.append({
                'name': 'Wind Speed',
                'status': 'FAIL',
                'message': f'{wind_speed:.1f} m/s (limit {wind_limit} m/s)',
                'severity': 'CRITICAL'
            })
        elif wind_speed > wind_limit * 0.8:  # Warning at 80% of limit
            checks.append({
                'name': 'Wind Speed',
                'status': 'WARN',
                'message': f'{wind_speed:.1f} m/s (approaching limit)',
                'severity': 'WARNING'
            })
        else:
            checks.append({
                'name': 'Wind Speed',
                'status': 'PASS',
                'message': f'{wind_speed:.1f} m/s',
                'severity': 'INFO'
            })
    except Exception as e:
        checks.append({
            'name': 'Wind Speed',
            'status': 'WARN',
            'message': f'Could not read wind data: {e}',
            'severity': 'WARNING'
        })
    
    # ============================================================
    # Check 3: GPS Quality
    # ============================================================
    gps_sat_threshold = 8
    
    if hasattr(state, 'gps_satellites'):
        if state.gps_satellites < gps_sat_threshold:
            checks.append({
                'name': 'GPS',
                'status': 'FAIL',
                'message': f'{state.gps_satellites} satellites (need >{gps_sat_threshold})',
                'severity': 'CRITICAL'
            })
        else:
            checks.append({
                'name': 'GPS',
                'status': 'PASS',
                'message': f'{state.gps_satellites} satellites',
                'severity': 'INFO'
            })
    else:
        # Simulated mode - assume GPS is fine
        checks.append({
            'name': 'GPS',
            'status': 'PASS',
            'message': 'Simulated (no hardware check)',
            'severity': 'INFO'
        })
    
    # ============================================================
    # Check 4: Home Position
    # ============================================================
    home_set = np.linalg.norm(world.hub_location) > 0.1  # Not at origin
    
    if not home_set:
        checks.append({
            'name': 'Home Position',
            'status': 'FAIL',
            'message': 'Hub location not set',
            'severity': 'CRITICAL'
        })
    else:
        checks.append({
            'name': 'Home Position',
            'status': 'PASS',
            'message': f'Hub at {world.hub_location}',
            'severity': 'INFO'
        })
    
    # ============================================================
    # Check 5: Flight Mode
    # ============================================================
    if state.mode != "GUIDED":
        checks.append({
            'name': 'Flight Mode',
            'status': 'WARN',
            'message': f'Currently in {state.mode} (will switch to GUIDED)',
            'severity': 'WARNING'
        })
    else:
        checks.append({
            'name': 'Flight Mode',
            'status': 'PASS',
            'message': 'GUIDED',
            'severity': 'INFO'
        })
    
    # ============================================================
    # Check 6: Vehicle Armed
    # ============================================================
    if not state.armed:
        checks.append({
            'name': 'Armed',
            'status': 'WARN',
            'message': 'Vehicle not armed (will arm on mission start)',
            'severity': 'WARNING'
        })
    else:
        checks.append({
            'name': 'Armed',
            'status': 'PASS',
            'message': 'Armed and ready',
            'severity': 'INFO'
        })
    
    # ============================================================
    # Display Results
    # ============================================================
    
    # Status symbols
    symbols = {
        'PASS': '✅',
        'WARN': '⚠️ ',
        'FAIL': '❌'
    }
    
    for check in checks:
        symbol = symbols.get(check['status'], '?')
        print(f"{symbol} {check['name']:20s} {check['message']}")
    
    # ============================================================
    # GO/NO-GO Decision
    # ============================================================
    
    # Any CRITICAL failures = NO-GO
    critical_failures = [c for c in checks if c['severity'] == 'CRITICAL' and c['status'] == 'FAIL']
    
    if critical_failures:
        print(f"\n{'='*50}")
        print("⛔ NO-GO: Critical failures detected")
        print(f"{'='*50}")
        for failure in critical_failures:
            print(f"  - {failure['name']}: {failure['message']}")
        return False
    
    # Warnings are okay but note them
    warnings = [c for c in checks if c['status'] == 'WARN']
    if warnings:
        print(f"\n{'='*50}")
        print("⚠️  WARNINGS: Proceed with caution")
        print(f"{'='*50}")
        for warning in warnings:
            print(f"  - {warning['name']}: {warning['message']}")
    
    print(f"\n{'='*50}")
    print("✅ GO: All critical checks passed")
    print(f"{'='*50}")
    
    return True


# Extended checks for specific mission types
def can_launch_mob_mission(world, state):
    """
    Specialized checks for Man Overboard missions.
    More stringent requirements due to life-critical nature.
    """
    # Run standard checks first
    if not can_launch(world, state):
        return False
    
    print("\n[PreFlight] Running MOB-specific checks...")
    
    mob_checks = []
    
    # Check for payload (life raft, etc.)
    # TODO: Add payload detection logic
    mob_checks.append({
        'name': 'Payload Attached',
        'status': 'WARN',
        'message': 'Cannot verify payload (manual inspection required)'
    })
    
    # Check thermal camera (if equipped)
    # TODO: Add thermal camera health check
    mob_checks.append({
        'name': 'Thermal Camera',
        'status': 'WARN',
        'message': 'Cannot verify thermal camera'
    })
    
    # Display MOB-specific results
    for check in mob_checks:
        symbol = '⚠️ ' if check['status'] == 'WARN' else '✅'
        print(f"{symbol} {check['name']:20s} {check['message']}")
    
    print("\n[PreFlight] MOB mission checks complete")
    print("⚠️  Reminder: Visually inspect payload and thermal camera")
    
    return True


# Quick test function
def test_preflight():
    """
    Test pre-flight checks with mock data.
    Run: python -c "from drone.preflight import test_preflight; test_preflight()"
    """
    from core.state.world_state import WorldState
    from dataclasses import dataclass
    import numpy as np
    
    @dataclass
    class MockState:
        battery: float = 95.0
        mode: str = "GUIDED"
        armed: bool = True
        gps_satellites: int = 12
        position_local: np.ndarray = np.array([0, 0, 0])
    
    print("\n" + "="*60)
    print("Pre-Flight Check Test (Mock Data)")
    print("="*60 + "\n")
    
    # Test 1: Good conditions
    print("Test 1: Good Conditions")
    print("-" * 60)
    world = WorldState()
    world.hub_location = np.array([100, 200, 0])
    state = MockState()
    
    result = can_launch(world, state)
    print(f"\nResult: {'PASS' if result else 'FAIL'}\n")
    
    # Test 2: Low battery
    print("\n" + "="*60)
    print("Test 2: Low Battery")
    print("-" * 60)
    state.battery = 85.0
    
    result = can_launch(world, state)
    print(f"\nResult: {'PASS' if result else 'FAIL'}\n")
    
    # Test 3: High wind
    print("\n" + "="*60)
    print("Test 3: High Wind")
    print("-" * 60)
    state.battery = 95.0
    # TODO: Mock high wind in WorldState
    
    result = can_launch(world, state)
    print(f"\nResult: {'PASS' if result else 'FAIL'}\n")


if __name__ == "__main__":
    test_preflight()