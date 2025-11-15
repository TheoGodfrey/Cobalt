# ship_configs.py - Complete system

SHIP_CONFIGS = {
    "small_fishing": {
        "type": "small",
        "deck_size": (10, 5),
        "landing_tolerance": 2.0,
        "max_heave": 3.0,
        "typical_speed": 2.5,
        "search_radius": 300,
        "return_altitude": 15,
        "motion_compensation": "aggressive",
        "approach_speed": 1.5,
        "battery_reserve": 30  # Higher for difficult landing
    },
    
    "medium_yacht": {
        "type": "medium",
        "deck_size": (20, 8),
        "landing_tolerance": 3.0,
        "max_heave": 1.5,
        "typical_speed": 7.5,
        "search_radius": 500,
        "return_altitude": 20,
        "motion_compensation": "moderate",
        "approach_speed": 2.5,
        "battery_reserve": 25
    },
    
    "large_cruise": {
        "type": "large",
        "deck_size": (100, 30),
        "landing_tolerance": 5.0,
        "max_heave": 0.5,
        "typical_speed": 10.0,
        "search_radius": 1000,
        "return_altitude": 30,
        "motion_compensation": "minimal",
        "approach_speed": 3.0,
        "battery_reserve": 25
    },
    
    # Easy to add new ship types
    "coast_guard_cutter": {
        "type": "medium",
        "deck_size": (30, 10),
        "landing_tolerance": 3.0,
        "max_heave": 1.2,
        "typical_speed": 12.0,
        "search_radius": 700,
        "return_altitude": 25,
        "motion_compensation": "moderate",
        "approach_speed": 2.5,
        "battery_reserve": 25
    }
}


def load_ship_config(ship_type_or_name):
    """
    Load ship config.
    
    Usage:
      ship = load_ship_config("small_fishing")
      ship = load_ship_config("large")  # Generic by type
    """
    if ship_type_or_name in SHIP_CONFIGS:
        return ShipConfig(SHIP_CONFIGS[ship_type_or_name])
    elif ship_type_or_name in ["small", "medium", "large"]:
        # Generic config
        generic = [c for c in SHIP_CONFIGS.values() 
                   if c["type"] == ship_type_or_name][0]
        return ShipConfig(generic)
    else:
        raise ValueError(f"Unknown ship: {ship_type_or_name}")