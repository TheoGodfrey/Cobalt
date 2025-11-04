"""
Mission Parser & Validator
Responsible for validating raw mission data against a schema and parsing
it into strongly-typed MissionFlow objects.
"""

from typing import Dict, Any
import jsonschema
from .mission_flow import MissionFlow, HubFailsafe
from .phase import Phase, Task

class MissionParseError(Exception):
    """Custom exception for errors during mission parsing or validation."""
    pass

# Defines the JSON schema for a valid mission file.
# This enforces the structure specified in COBALT_Architecture_Specification_4.md.
MISSION_SCHEMA = {
    "type": "object",
    "required": ["mission_id", "start_phase", "phases"],
    "properties": {
        "mission_id": {"type": "string", "pattern": "^[a-z0-9_]+$"},
        "mission_name": {"type": "string"},
        "is_alterable": {"type": "boolean", "default": True},
        "hub_failsafe": {
            "type": "string",
            "enum": ["RTH", "CONTINUE", "LAND"],
            "default": "RTH"
        },
        "start_phase": {"type": "string"},
        "phases": {
            "type": "object",
            "patternProperties": {
                "^[a-z0-9_]+$": {"$ref": "#/definitions/Phase"}
            },
            "minProperties": 1
        }
    },
    "definitions": {
        "Phase": {
            "type": "object",
            "required": ["tasks"],
            "properties": {
                "tasks": {
                    "type": "object",
                    "patternProperties": {
                        "^[a-z0-9_]+$": {"$ref": "#/definitions/Task"}
                    },
                    "minProperties": 1
                },
                "transitions": {
                    "type": "object",
                    "patternProperties": {
                        "^on_(event|state|timeout):[a-z0-9_.]+$": { # Allow timeout with float
                            "type": "object",
                            "patternProperties": {
                                "^[a-z0-9_]+$": {"type": "string", "pattern": "^goto:[a-z0-9_]+$"}
                            },
                            "minProperties": 1
                        }
                    }
                }
            }
        },
        "Task": {
            "type": "object",
            "required": ["action"],
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["EXECUTE_SEARCH", "EXECUTE_DELIVERY", "EXECUTE_STANDBY", "EXECUTE_RTH", "IGNORE", "EXECUTE_PATROL", "EXECUTE_OVERWATCH"]
                },
                "detector": {"type": "string"},
                "strategy": {"type": "string"},
                "actuator": {"type": "string"},
                # --- FIX: Allow a 'params' object in the JSON ---
                "params": {
                    "type": "object",
                    "additionalProperties": { "type": "object" }
                }
                # --- End of FIX ---
            }
        }
    }
}

def validate_mission_data(data: Dict[str, Any]) -> None:
    """
    Validates a raw mission dictionary against the MISSION_SCHEMA.

    Args:
        data: The raw dictionary loaded from JSON.

    Raises:
        MissionParseError: If validation fails.
    """
    try:
        # --- FIX for Bug #15 ---
        # The original code had a non-functional loop and validated twice.
        # The correct behavior is to apply defaults *before* validating.
        
        # Manually apply defaults for root properties (as defined in schema)
        if 'is_alterable' not in data:
            data['is_alterable'] = MISSION_SCHEMA['properties']['is_alterable']['default']
        if 'hub_failsafe' not in data:
            data['hub_failsafe'] = MISSION_SCHEMA['properties']['hub_failsafe']['default']

        # Now, validate the data (with defaults applied) against the schema.
        # This single call checks for required fields, types, enums, etc.
        jsonschema.validate(instance=data, schema=MISSION_SCHEMA)

    except jsonschema.ValidationError as e:
        raise MissionParseError(f"Mission schema validation failed: {e.message}")
    except Exception as e:
        raise MissionParseError(f"An unexpected error occurred during validation: {e}")

def parse_mission_flow(data: Dict[str, Any]) -> MissionFlow:
    """
    Parses a validated raw mission dictionary into a MissionFlow object.

    Args:
        data: The raw dictionary, assumed to be validated.

    Returns:
        A strongly-typed MissionFlow object.
    
    Raises:
        MissionParseError: If parsing fails.
    """
    try:
        # 1. Validate the raw data first
        validate_mission_data(data)

        # 2. Parse Phases and Tasks (bottom-up)
        parsed_phases: Dict[str, Phase] = {}
        for phase_name, phase_data in data["phases"].items():
            parsed_tasks: Dict[str, Task] = {}
            for role_name, task_data in phase_data["tasks"].items():
                parsed_tasks[role_name] = Task(
                    action=task_data["action"],
                    detector=task_data.get("detector"),
                    strategy=task_data.get("strategy"),
                    actuator=task_data.get("actuator"),
                    # --- FIX: Read the params from the JSON ---
                    params=task_data.get("params", {})
                    # --- End of FIX ---
                )
            
            parsed_phases[phase_name] = Phase(
                tasks=parsed_tasks,
                transitions=phase_data.get("transitions", {})
            )

        # 3. Create the final MissionFlow object
        mission_flow = MissionFlow(
            mission_id=data["mission_id"],
            is_alterable=data["is_alterable"],
            hub_failsafe=HubFailsafe(data["hub_failsafe"]),
            start_phase=data["start_phase"],
            phases=parsed_phases
        )

        # 4. Final semantic check: does start_phase exist?
        if mission_flow.start_phase not in mission_flow.phases:
            raise MissionParseError(f"start_phase '{mission_flow.start_phase}' not found in 'phases' dictionary.")

        return mission_flow

    except KeyError as e:
        raise MissionParseError(f"Missing expected key during parsing: {e}")
    except Exception as e:
        raise MissionParseError(f"An error occurred during object construction: {e}")
