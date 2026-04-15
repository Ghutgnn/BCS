from __future__ import annotations

from sim_compare.bridges.esmini import EsminiBridge
from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState
from sim_compare.simulators.base import SimulatorDescriptor, make_csv_prefix


class EsminiSimulatorAdapter:
    def __init__(self, bridge: EsminiBridge, label: str = "esmini"):
        self.bridge = bridge
        self.descriptor = SimulatorDescriptor(
            simulator_id="esmini",
            label=label,
            csv_prefix=make_csv_prefix(label),
            control_space=bridge.control_space,
            backend_name=bridge.backend_name,
        )

    def start(self, initial_pose: InitialPose) -> None:
        self.bridge.set_initial_pose(initial_pose)
        self.bridge.start()

    def get_measurement_start_pose(self) -> InitialPose | None:
        return None

    def get_initialization_notes(self) -> list[str]:
        return []

    def step(
        self,
        control: AppliedControlCommand,
        dt_s: float,
        timestamp_s: float,
    ) -> VehicleState:
        return self.bridge.step(control, dt_s, timestamp_s)

    def get_state(self, timestamp_s: float) -> VehicleState:
        return self.bridge.get_state(timestamp_s)

    def should_quit(self) -> bool:
        return self.bridge.should_quit()

    def get_render_actor(self):
        return None

    def close(self) -> None:
        self.bridge.close()
