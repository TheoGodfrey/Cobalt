"""
Component 2: Phase
Defines the data structures for mission phases and their constituent tasks.
A Phase is a reusable building block of a mission.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict

@dataclass(frozen=True)
class Task:
    """
    Defines a specific action for a drone role within a phase.
    Based on the 'Task' schema in COBALT_Architecture_Specification_4.md.
    """
    # The action to perform, e.g., "EXECUTE_SEARCH", "EXECUTE_DELIVERY"
    action: str
    
    # Optional plugin specifications
    detector: Optional[str] = None
    strategy: Optional[str] = None
    actuator: Optional[str] = None

@dataclass(frozen=True)
class Phase:
    """
    Defines a single, reusable mission phase.
    Based on the 'Phase' definition in COBALT_Architecture_Specification_4.md.
    """
    # Maps a role (e.g., "scout", "payload") to its specific Task
    tasks: Dict[str, Task]
    
    # Defines the transitions out of this phase.
    # Key: The trigger, e.g., "on_event:target_found"
    # Value: A dict mapping role to destination, e.g., {"scout": "goto:delivery"}
    transitions: Dict[str, Dict[str, str]] = field(default_factory=dict)
