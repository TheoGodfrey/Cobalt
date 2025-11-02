"""
Strategy registry and factory (composition-based)
(Updated to accept Pydantic config objects)
"""

# Registry of available strategies
_FLIGHT_STRATEGIES = {}
_SEARCH_STRATEGIES = {}

def register_flight_strategy(name: str, strategy_factory):
    _FLIGHT_STRATEGIES[name] = strategy_factory

def register_search_strategy(name: str, strategy_factory):
    _SEARCH_STRATEGIES[name] = strategy_factory

# CHANGED: Now accepts a config object
def get_flight_strategy(name: str, config: object):
    """Get flight strategy by name - passes config to factory"""
    if name not in _FLIGHT_STRATEGIES:
        raise ValueError(f"Unknown flight strategy: {name}")
    # Pass the specific config object to the factory
    return _FLIGHT_STRATEGIES[name](config)

# CHANGED: Now accepts a config object
def get_search_strategy(name: str, config: object):
    """Get search strategy by name - passes config to factory"""
    if name not in _SEARCH_STRATEGIES:
        raise ValueError(f"Unknown search strategy: {name}")
    # Pass the specific config object to the factory
    return _SEARCH_STRATEGIES[name](config)

def list_available_strategies():
    """List all available strategies"""
    return {
        "flight": list(_FLIGHT_STRATEGIES.keys()),
        "search": list(_SEARCH_STRATEGIES.keys())
    }

# Auto-register strategies using factory functions
from .flight.direct import create_direct_flight_strategy
from .flight.precision_hover import create_precision_hover_flight_strategy
from .search.random import create_random_search_strategy
from .search.vertical_ascent import create_vertical_ascent_search_strategy

from .search.lawnmower import create_lawnmower_search_strategy
from .flight.orbit import create_orbit_flight_strategy

register_flight_strategy('direct', create_direct_flight_strategy)
register_flight_strategy('precision_hover', create_precision_hover_flight_strategy)
register_search_strategy('random', create_random_search_strategy)
register_search_strategy('vertical_ascent', create_vertical_ascent_search_strategy)

# NEW: Register lawnmower and orbit
register_search_strategy('lawnmower', create_lawnmower_search_strategy)
register_flight_strategy('orbit', create_orbit_flight_strategy)