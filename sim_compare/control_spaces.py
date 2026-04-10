from __future__ import annotations

CANONICAL_CONTROL_SPACE = "canonical.actuation"
CARLA_CONTROL_SPACE = "carla.vehicle"
ESMINI_SIMPLE_CONTROL_SPACE = "esmini.simple_vehicle"
ESMINI_BCS_CONTROL_SPACE = "esmini.bcs_controller"


SUPPORTED_CONTROL_SPACES = {
    CANONICAL_CONTROL_SPACE,
    CARLA_CONTROL_SPACE,
    ESMINI_SIMPLE_CONTROL_SPACE,
    ESMINI_BCS_CONTROL_SPACE,
}


def normalize_control_space(control_space: str) -> str:
    value = control_space.strip().lower()
    aliases = {
        "canonical": CANONICAL_CONTROL_SPACE,
        "canonical.actuation": CANONICAL_CONTROL_SPACE,
        "carla": CARLA_CONTROL_SPACE,
        "carla.vehicle": CARLA_CONTROL_SPACE,
        "esmini.simple": ESMINI_SIMPLE_CONTROL_SPACE,
        "esmini.simple_vehicle": ESMINI_SIMPLE_CONTROL_SPACE,
        "simple_vehicle_api": ESMINI_SIMPLE_CONTROL_SPACE,
        "esmini.bcs": ESMINI_BCS_CONTROL_SPACE,
        "esmini.bcs_controller": ESMINI_BCS_CONTROL_SPACE,
        "bcs_controller": ESMINI_BCS_CONTROL_SPACE,
        "udp_driver_controller": ESMINI_BCS_CONTROL_SPACE,
    }
    normalized = aliases.get(value, value)
    if normalized not in SUPPORTED_CONTROL_SPACES:
        raise ValueError(f"Unsupported control space: {control_space}")
    return normalized
