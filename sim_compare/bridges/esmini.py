from __future__ import annotations

from pathlib import Path
from typing import Protocol

from sim_compare.bridges.esmini_runtime import EsminiRuntime
from sim_compare.bridges.esmini_bcs_controller import EsminiBCSControllerBridge
from sim_compare.bridges.esmini_simple_vehicle import EsminiSimpleVehicleBridge
from sim_compare.config import EsminiBackendConfig, EsminiOptions
from sim_compare.esmini_backend import normalize_esmini_backend_mode
from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState
from sim_compare.simple_vehicle_config import load_simple_vehicle_config


class EsminiBridge(Protocol):
    backend_name: str
    simulator_name: str
    control_space: str

    def start(self) -> None:
        ...

    def set_initial_pose(self, pose: InitialPose) -> None:
        ...

    def step(
        self,
        control: AppliedControlCommand,
        dt_s: float,
        timestamp_s: float,
    ) -> VehicleState:
        ...

    def get_state(self, timestamp_s: float) -> VehicleState:
        ...

    def should_quit(self) -> bool:
        ...

    def close(self) -> None:
        ...


def create_esmini_bridge(
    esmini_home: Path,
    xosc_path: Path,
    search_paths: list[Path],
    ego_index: int,
    options: EsminiOptions,
    backend: EsminiBackendConfig,
) -> EsminiBridge:
    normalized_mode = normalize_esmini_backend_mode(backend.mode)
    runtime = EsminiRuntime(
        esmini_home=esmini_home,
        search_paths=search_paths,
        ego_index=ego_index,
        options=options,
    )

    if normalized_mode == "simple_vehicle_api":
        vehicle_config = None
        if backend.simple_vehicle_config_path is not None:
            vehicle_config = load_simple_vehicle_config(
                backend.simple_vehicle_config_path
            )
        return EsminiSimpleVehicleBridge(
            runtime=runtime,
            xosc_path=xosc_path,
            vehicle_config=vehicle_config,
        )
    if normalized_mode == "bcs_controller":
        return EsminiBCSControllerBridge(
            runtime=runtime,
            xosc_path=xosc_path,
            base_port=backend.udp_base_port,
            exec_mode=backend.udp_exec_mode,
        )
    raise ValueError(f"Unsupported esmini backend: {backend.mode}")
