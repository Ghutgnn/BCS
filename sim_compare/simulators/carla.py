from __future__ import annotations

from sim_compare.bridges.carla import CarlaBridge
from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState
from sim_compare.simulators.base import SimulatorDescriptor, make_csv_prefix


class CarlaSimulatorAdapter:
    def __init__(self, bridge: CarlaBridge, label: str = "carla"):
        self.bridge = bridge
        self.descriptor = SimulatorDescriptor(
            simulator_id="carla",
            label=label,
            csv_prefix=make_csv_prefix(label),
            control_space=bridge.control_space,
            backend_name="native",
        )

    def start(self, initial_pose: InitialPose) -> None:
        self.bridge.spawn_vehicle(initial_pose)

    def get_measurement_start_pose(self) -> InitialPose | None:
        return self.bridge.get_measurement_start_pose()

    def get_initialization_notes(self) -> list[str]:
        ticks = self.bridge.get_last_readiness_ticks()
        if ticks <= 0:
            return []
        return [f"carla readiness gate consumed {ticks} non-recorded ticks"]

    def step(
        self,
        control: AppliedControlCommand,
        dt_s: float,
        timestamp_s: float,
    ) -> VehicleState:
        del dt_s
        return self.bridge.step(control, timestamp_s)

    def get_state(self, timestamp_s: float) -> VehicleState:
        return self.bridge.get_state(timestamp_s)

    def should_quit(self) -> bool:
        return False

    def get_render_actor(self):
        return self.bridge.actor

    def close(self) -> None:
        self.bridge.close()
