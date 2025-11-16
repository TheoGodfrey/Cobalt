# track_1/ship_config.py
# (This is your uploaded 'ship_configs.py', renamed for consistency
# and with the ShipConfig class added from our discussion)

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

class ShipConfig:
    """
    Loads ship config from the dictionary and provides dot-notation access.
    """
    def __init__(self, ship_type_or_name: str):
        
        config_data = None
        if ship_type_or_name in SHIP_CONFIGS:
            config_data = SHIP_CONFIGS[ship_type_or_name]
        elif ship_type_or_name in ["small", "medium", "large"]:
             # Generic config
            for cfg in SHIP_CONFIGS.values():
                if cfg["type"] == ship_type_or_name:
                    config_data = cfg
                    break
        
        if not config_data:
            raise ValueError(f"Unknown ship: {ship_type_or_name}")
            
        self.ship_type = ship_type_or_name
        # Dynamically assign all keys from the config dict
        for key, value in config_data.items():
            setattr(self, key, value)
            
        print(f"[Config] Loaded ship config: {self.ship_type}")