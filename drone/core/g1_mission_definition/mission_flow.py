"""
Component 1: Mission Flow
Defines the top-level data structure for a complete mission campaign.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict
from .phase import Phase  # Import from sibling file

class HubFailsafe(str, Enum):
    """
    Defines the autonomous action to take if the hub connection is lost.
    Based on 'hub_failsafe' in COBALT_Architecture_Specification_4.md.
    """
    RTH = "RTH"
    CONTINUE = "CONTINUE"
    LAND = "LAND"

@dataclass(frozen=True)
class MissionFlow:
    """
    Represents the complete mission campaign structure, loaded from JSON.
    Based on 'Mission Flow' in COBALT_Architecture_Specification_4.md.
    """
    mission_id: str
    is_alterable: bool
    hub_failsafe: HubFailsafe
    start_phase: str
    phases: Dict[str, Phase]
