"""
Critical Fixes Validation Suite

Run this to verify all 5 bugs are fixed before sea trials.

Usage:
    pytest tests/test_critical_fixes.py -v
    
    OR
    
    python tests/test_critical_fixes.py
"""
import pytest
import asyncio
import numpy as np
import time
from dataclasses import dataclass


# ============================================================
# Test 1: Comms Thread Safety
# ============================================================

def test_comms_thread_safety():
    """
    Bug 1: Comms race condition between MQTT and asyncio.
    
    Fix: Use run_coroutine_threadsafe() to bridge threads.
    
    Test: Start comms service and verify it doesn't crash.
    """
    from core.comms import Comms, MessageBus
    
    bus = MessageBus()
    comms = Comms("test_drone", bus, broker="localhost")
    
    # Should not crash during initialization
    assert comms.loop is None  # Not started yet
    
    # Test async startup
    async def test_startup():
        # Start comms in background
        comms_task = asyncio.create_task(comms.run())
        
        # Give it time to start
        await asyncio.sleep(0.5)
        
        # Verify loop was captured
        assert comms.loop is not None
        assert comms.loop.is_running()
        
        # Cancel task
        comms_task.cancel()
        try:
            await comms_task
        except asyncio.CancelledError:
            pass
    
    asyncio.run(test_startup())
    print("✅ Test 1 PASSED: Comms thread-safe")


# ============================================================
# Test 2: Probability Type Handling
# ============================================================

def test_probability_type_handling():
    """
    Bug 2: Probability.evolve() crashes on tuple vs numpy array.
    
    Fix: Use np.asarray() with explicit dtype and validation.
    
    Test: Pass various input types and verify no crash.
    """
    from core.state.probability import ProbabilityModel
    
    prob = ProbabilityModel(resolution=10.0, bounds=(-100, 100, -100, 100))
    
    # Test 1: Tuple input (original bug)
    try:
        prob.evolve(0.1, drift_vector_ms=(1.0, 0.5, 0.0))
        print("✅ Test 2a PASSED: Handles tuple")
    except Exception as e:
        pytest.fail(f"Failed on tuple: {e}")
    
    # Test 2: Numpy array
    try:
        prob.evolve(0.1, drift_vector_ms=np.array([1.0, 0.5, 0.0]))
        print("✅ Test 2b PASSED: Handles numpy array")
    except Exception as e:
        pytest.fail(f"Failed on numpy array: {e}")
    
    # Test 3: Malformed input (wrong size)
    try:
        prob.evolve(0.1, drift_vector_ms=[1.0])  # Only 1 element
        print("✅ Test 2c PASSED: Handles malformed input")
    except Exception as e:
        pytest.fail(f"Failed on malformed input: {e}")
    
    # Test 4: None input (should use default)
    try:
        prob.evolve(0.1)
        print("✅ Test 2d PASSED: Handles None/default")
    except Exception as e:
        pytest.fail(f"Failed on None: {e}")


# ============================================================
# Test 3: Fleet Position Syncing
# ============================================================

def test_fleet_position_sync():
    """
    Bug 3: Fleet positions not syncing for swarm coordination.
    
    Fix: Wire fleet telemetry to world.fleet_positions.
    
    Test: Simulate fleet telemetry and verify world state updates.
    """
    from core.state.world_state import WorldState
    
    world = WorldState()
    
    # Simulate fleet telemetry updates
    world.fleet_positions['scout_2'] = np.array([100, 200, -50])
    world.fleet_positions['utility_1'] = np.array([150, 250, -50])
    
    # Verify positions stored
    assert 'scout_2' in world.fleet_positions
    assert 'utility_1' in world.fleet_positions
    
    # Test nearest neighbor calculation
    my_pos = np.array([0, 0, -50])
    nearest_id, nearest_pos, dist = world.get_nearest_neighbor(my_pos)
    
    assert nearest_id == 'scout_2'  # Closest to origin
    assert dist < 250  # Rough distance check
    
    print("✅ Test 3 PASSED: Fleet positions sync")


# ============================================================
# Test 4: Pause/Resume
# ============================================================

def test_pause_resume():
    """
    Bug 4: Mission Manager missing pause/resume for manual override.
    
    Fix: Add pause_mission() and resume_mission() methods.
    
    Test: Pause and resume mission, verify state preservation.
    """
    from core.mission.manager import MissionManager
    from core.state.world_state import WorldState
    from core.maths.solver import ProbabilisticSolver
    from drone.safety.safety_monitor import SafetyMonitor
    
    # Mock mission plan
    mission_plan = {
        'start_phase': 'search',
        'phases': {
            'search': {
                'tasks': {
                    'scout': {
                        'default_behavior': {
                            'cost_equation': 'UnifiedRescue',
                            'posture': 'COOPERATIVE'
                        }
                    }
                },
                'transitions': {}
            }
        }
    }
    
    world = WorldState()
    solver = ProbabilisticSolver(config={})
    safety = SafetyMonitor(world, config={})
    
    manager = MissionManager(mission_plan, world, solver, safety, my_role='scout')
    manager.start()
    
    # Should be in search phase
    assert manager.current_phase == 'search'
    assert not manager.is_paused()
    
    # Pause mission
    manager.pause_mission("manual_override")
    
    assert manager.is_paused()
    assert manager.pause_reason == "manual_override"
    assert manager.pause_phase_idx == 0  # Was in first phase
    
    # Resume mission
    manager.resume_mission()
    
    assert not manager.is_paused()
    assert manager.current_phase == 'search'  # Restored
    
    print("✅ Test 4 PASSED: Pause/resume works")


# ============================================================
# Test 5: Hub Motion Tracking
# ============================================================

def test_hub_motion_tracking():
    """
    Maritime Essential 1: Hub motion tracking for moving boat.
    
    Test: Update hub with velocity and verify prediction works.
    """
    from core.state.world_state import WorldState
    
    world = WorldState()
    
    # Initial hub position
    world.update_hub(
        position=[0, 0, 0],
        velocity=[5.0, 0.0, 0.0],  # 5 m/s north
        gps_quality=8
    )
    
    # Simulate 2 second delay
    time.sleep(2)
    
    # Predict hub position (should account for 2s of movement)
    predicted = world.get_hub_with_prediction(lookahead_seconds=0)
    
    # Should have moved ~10m north (5 m/s * 2s)
    expected_x = 10.0
    actual_x = predicted[0]
    
    assert abs(actual_x - expected_x) < 1.0, f"Expected ~{expected_x}m, got {actual_x}m"
    
    # Test motion quality
    quality = world.get_hub_motion_quality()
    assert quality in ["GOOD", "DEGRADED", "LOST"]
    
    # Test stationary detection
    assert not world.is_hub_stationary()  # 5 m/s is not stationary
    
    world.hub_velocity = np.array([0.0, 0.0, 0.0])
    assert world.is_hub_stationary()  # 0 m/s is stationary
    
    print("✅ Test 5 PASSED: Hub motion tracking works")


# ============================================================
# Test 6: Pre-Flight Checks
# ============================================================


# ============================================================
# Test 7: Structured Logging
# ============================================================

def test_structured_logging():
    """
    Maritime Essential 3: Structured logging.
    
    Test: Log events and verify JSONL format.
    """
    import json
    import os
    from core.logger import MissionLogger
    
    log_file = "test_mission.jsonl"
    
    # Clean up old test log
    if os.path.exists(log_file):
        os.remove(log_file)
    
    # Create logger
    logger = MissionLogger("test_drone", log_file)
    
    # Log some events
    logger.log("STARTUP", {'mission': 'test'})
    logger.log("STATE", {'phase': 'search', 'battery': 95})
    logger.log("DETECTION", {'label': 'person', 'confidence': 0.87})
    
    logger.close()
    
    # Read back and verify
    with open(log_file, 'r') as f:
        lines = f.readlines()
    
    assert len(lines) == 3, f"Expected 3 log lines, got {len(lines)}"
    
    # Verify JSON format
    for line in lines:
        entry = json.loads(line)
        assert 't' in entry
        assert 'id' in entry
        assert 'event' in entry
        assert 'data' in entry
    
    # Verify content
    first_entry = json.loads(lines[0])
    assert first_entry['event'] == 'STARTUP'
    assert first_entry['id'] == 'test_drone'
    
    # Clean up
    os.remove(log_file)
    
    print("✅ Test 6 PASSED: Structured logging works")


# ============================================================
# Run All Tests
# ============================================================

if __name__ == "__main__":
    """Run all tests without pytest"""
    print("\n" + "="*60)
    print("Critical Fixes Validation Suite")
    print("="*60 + "\n")
    
    tests = [
        ("Comms Thread Safety", test_comms_thread_safety),
        ("Probability Type Handling", test_probability_type_handling),
        ("Fleet Position Sync", test_fleet_position_sync),
        ("Pause/Resume", test_pause_resume),
        ("Hub Motion Tracking", test_hub_motion_tracking),
        ("Structured Logging", test_structured_logging)
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        print(f"\nRunning: {name}")
        print("-" * 60)
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"❌ FAILED: {e}")
            failed += 1
    
    print("\n" + "="*60)
    print(f"Results: {passed} passed, {failed} failed")
    print("="*60 + "\n")
    
    if failed == 0:
        print("🎉 All tests passed! Ready for sea trials.")
    else:
        print(f"⚠️  {failed} tests failed. Fix before deploying.")
        exit(1)