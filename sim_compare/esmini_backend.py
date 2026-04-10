from __future__ import annotations

from sim_compare.control_spaces import (
    ESMINI_BCS_CONTROL_SPACE,
    ESMINI_SIMPLE_CONTROL_SPACE,
)

ESMINI_SIMPLE_VEHICLE_BACKEND = "simple_vehicle_api"
ESMINI_BCS_BACKEND = "bcs_controller"
ESMINI_UDP_ALIAS_BACKEND = "udp_driver_controller"

SUPPORTED_ESMINI_BACKENDS = {
    ESMINI_SIMPLE_VEHICLE_BACKEND,
    ESMINI_BCS_BACKEND,
    ESMINI_UDP_ALIAS_BACKEND,
}


def normalize_esmini_backend_mode(mode: str) -> str:
    value = mode.strip().lower()
    if value == ESMINI_UDP_ALIAS_BACKEND:
        return ESMINI_BCS_BACKEND
    if value not in SUPPORTED_ESMINI_BACKENDS:
        raise ValueError(f"Unsupported esmini backend: {mode}")
    return value


def esmini_backend_to_control_space(mode: str) -> str:
    normalized = normalize_esmini_backend_mode(mode)
    if normalized == ESMINI_SIMPLE_VEHICLE_BACKEND:
        return ESMINI_SIMPLE_CONTROL_SPACE
    if normalized == ESMINI_BCS_BACKEND:
        return ESMINI_BCS_CONTROL_SPACE
    raise ValueError(f"Unsupported esmini backend: {mode}")
