from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Protocol

from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState


@dataclass(frozen=True)
class SimulatorDescriptor:
    simulator_id: str
    label: str
    csv_prefix: str
    control_space: str
    backend_name: str = "native"


class SimulatorAdapter(Protocol):
    descriptor: SimulatorDescriptor

    def start(self, initial_pose: InitialPose) -> None:
        ...

    def get_measurement_start_pose(self) -> InitialPose | None:
        ...

    def get_initialization_notes(self) -> list[str]:
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

    def get_render_actor(self):
        ...

    def close(self) -> None:
        ...


def make_csv_prefix(label: str) -> str:
    normalized = re.sub(r"[^0-9a-zA-Z_]+", "_", label.strip().lower())
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    if not normalized:
        raise ValueError(f"Unable to build CSV prefix from label: {label!r}")
    return normalized
