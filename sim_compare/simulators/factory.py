from __future__ import annotations

from collections.abc import Callable

from sim_compare.bridges.carla import CarlaBridge
from sim_compare.bridges.esmini import create_esmini_bridge
from sim_compare.config import ExperimentConfig
from sim_compare.maps import MapPaths
from sim_compare.simulators.base import SimulatorAdapter
from sim_compare.simulators.carla import CarlaSimulatorAdapter
from sim_compare.simulators.esmini import EsminiSimulatorAdapter


SUPPORTED_SIMULATORS = frozenset({"carla", "esmini"})


def normalize_simulator_id(simulator_id: str) -> str:
    key = simulator_id.strip().lower()
    aliases = {
        "carla": "carla",
        "esmini": "esmini",
    }
    normalized = aliases.get(key)
    if normalized is None:
        raise ValueError(f"Unsupported simulator: {simulator_id}")
    return normalized


def _build_carla_adapter(
    config: ExperimentConfig,
    map_paths: MapPaths,
    search_paths,
    label: str,
) -> SimulatorAdapter:
    del search_paths
    bridge = CarlaBridge(
        host=config.carla_host,
        port=config.carla_port,
        timeout_s=config.carla_timeout,
        traffic_manager_port=config.traffic_manager_port,
        vehicle_filter=config.carla_vehicle_filter,
        xodr_path=map_paths.xodr_path,
        dt_s=config.dt,
        coordinate_transform=config.coordinate_transform,
        opendrive_config=config.carla_opendrive,
    )
    return CarlaSimulatorAdapter(bridge=bridge, label=label)


def _build_esmini_adapter(
    config: ExperimentConfig,
    map_paths: MapPaths,
    search_paths,
    label: str,
) -> SimulatorAdapter:
    bridge = create_esmini_bridge(
        esmini_home=config.esmini_home,
        xosc_path=map_paths.xosc_path,
        search_paths=search_paths,
        ego_index=config.ego_index,
        options=config.esmini_options,
        backend=config.esmini_backend,
    )
    return EsminiSimulatorAdapter(bridge=bridge, label=label)


SIMULATOR_BUILDERS: dict[str, Callable[..., SimulatorAdapter]] = {
    "carla": _build_carla_adapter,
    "esmini": _build_esmini_adapter,
}


def build_simulator_adapter(
    simulator_id: str,
    config: ExperimentConfig,
    map_paths: MapPaths,
    search_paths,
    label: str | None = None,
) -> SimulatorAdapter:
    normalized_id = normalize_simulator_id(simulator_id)
    builder = SIMULATOR_BUILDERS[normalized_id]
    return builder(config, map_paths, search_paths, label or normalized_id)
