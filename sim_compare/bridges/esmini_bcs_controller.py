from __future__ import annotations

import socket
import struct
from pathlib import Path

from sim_compare.control_spaces import ESMINI_BCS_CONTROL_SPACE
from sim_compare.bridges.esmini_runtime import EsminiRuntime
from sim_compare.bridges.esmini_scenario import build_bcs_runtime_xosc
from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState


class EsminiBCSControllerBridge:
    backend_name = "bcs_controller"
    simulator_name = "esmini"
    control_space = ESMINI_BCS_CONTROL_SPACE
    _MESSAGE_FORMAT = "iiiiddd"
    _INPUT_MODE_DRIVER = 1
    _MESSAGE_VERSION = 1

    def __init__(
        self,
        runtime: EsminiRuntime,
        xosc_path: Path,
        base_port: int,
        exec_mode: str,
    ):
        self.runtime = runtime
        self.base_xosc_path = xosc_path
        self.base_port = base_port
        self.exec_mode = exec_mode
        self.initial_pose: InitialPose | None = None
        self.runtime_xosc_path = xosc_path.with_name(
            f"{xosc_path.stem}.runtime_bcs.xosc"
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_number = 0
        self._warned_reverse = False

    def start(self) -> None:
        if self.initial_pose is None:
            raise RuntimeError(
                "Initial pose must be set before starting BCS controller backend"
            )

        build_bcs_runtime_xosc(
            self.base_xosc_path,
            self.runtime_xosc_path,
            self.initial_pose,
            self.base_port,
            self.exec_mode,
        )
        self.runtime.start(self.runtime_xosc_path, disable_controller=0)

    def set_initial_pose(self, pose: InitialPose) -> None:
        self.initial_pose = pose

    def _send_driver_input(self, control: AppliedControlCommand) -> None:
        if control.reverse and not self._warned_reverse:
            print(
                "Warning: esmini BCSController driverInput does not expose reverse gear; "
                "reverse commands are ignored on the esmini side."
            )
            self._warned_reverse = True

        payload = struct.pack(
            self._MESSAGE_FORMAT,
            self._MESSAGE_VERSION,
            self._INPUT_MODE_DRIVER,
            int(self.runtime.obj_id),
            int(self.frame_number),
            float(control.throttle),
            float(control.brake),
            float(control.steering_angle_rad),
        )
        self.sock.sendto(payload, ("127.0.0.1", self.base_port + self.runtime.obj_id))
        self.frame_number += 1

    def step(
        self,
        control: AppliedControlCommand,
        dt_s: float,
        timestamp_s: float,
    ) -> VehicleState:
        self._send_driver_input(control)
        ret = self.runtime.se.SE_StepDT(float(dt_s))
        if ret != 0:
            raise RuntimeError(f"SE_StepDT failed in BCS controller backend, ret={ret}")
        return self.runtime.get_ego_state(timestamp_s)

    def get_state(self, timestamp_s: float) -> VehicleState:
        return self.runtime.get_ego_state(timestamp_s)

    def should_quit(self) -> bool:
        return self.runtime.should_quit()

    def close(self) -> None:
        try:
            self.sock.close()
        finally:
            self.runtime.close()


# Backward-compatible alias for older imports.
EsminiUdpDriverBridge = EsminiBCSControllerBridge
