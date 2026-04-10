from __future__ import annotations

import csv
import math
from pathlib import Path

from sim_compare.models import (
    AppliedControlCommand,
    InputControlCommand,
    VehicleState,
)
from sim_compare.simulators.base import SimulatorDescriptor
from sim_compare.utils import normalize_yaw_rad


INPUT_CONTROL_FIELDS = [
    "input_control_throttle",
    "input_control_brake",
    "input_control_steer",
    "input_control_hand_brake",
    "input_control_reverse",
]
COMPARISON_METADATA_FIELDS = [
    "step",
    "sim_time_s",
    *INPUT_CONTROL_FIELDS,
    "control_source_space",
    "control_mapping_strategy",
    "reference_simulator_id",
    "reference_label",
    "reference_csv_prefix",
    "reference_backend",
    "candidate_simulator_id",
    "candidate_label",
    "candidate_csv_prefix",
    "candidate_backend",
]
CONTROL_SUFFIXES = [
    "control_throttle",
    "control_brake",
    "control_steer",
    "control_pedal",
    "control_steering_angle_rad",
    "control_space",
    "control_hand_brake",
    "control_reverse",
]
STATE_SUFFIXES = [
    "x",
    "y",
    "z",
    "yaw",
    "speed",
    "acceleration",
    "vel_x",
    "vel_y",
    "vel_z",
    "ax",
    "ay",
    "az",
    "wheel_angle",
    "wheel_rotation",
]
DIFF_FIELDS = [
    "diff_x",
    "diff_y",
    "diff_z",
    "diff_yaw",
    "diff_speed",
    "diff_acceleration",
    "diff_pos_2d",
    "diff_pos_3d",
]


def _prefixed_fields(prefix: str, suffixes: list[str]) -> list[str]:
    return [f"{prefix}_{suffix}" for suffix in suffixes]


def build_csv_fields(
    reference_prefix: str,
    candidate_prefix: str,
) -> list[str]:
    return [
        *COMPARISON_METADATA_FIELDS,
        *_prefixed_fields(reference_prefix, CONTROL_SUFFIXES),
        *_prefixed_fields(candidate_prefix, CONTROL_SUFFIXES),
        *_prefixed_fields(reference_prefix, STATE_SUFFIXES),
        *_prefixed_fields(candidate_prefix, STATE_SUFFIXES),
        *DIFF_FIELDS,
    ]


def compute_state_differences(
    reference_state: VehicleState,
    candidate_state: VehicleState,
) -> dict[str, float]:
    diff_x = reference_state.x - candidate_state.x
    diff_y = reference_state.y - candidate_state.y
    diff_z = reference_state.z - candidate_state.z
    return {
        "diff_x": diff_x,
        "diff_y": diff_y,
        "diff_z": diff_z,
        "diff_yaw": normalize_yaw_rad(reference_state.yaw - candidate_state.yaw),
        "diff_speed": reference_state.speed - candidate_state.speed,
        "diff_acceleration": (
            reference_state.acceleration - candidate_state.acceleration
        ),
        "diff_pos_2d": math.sqrt(diff_x * diff_x + diff_y * diff_y),
        "diff_pos_3d": math.sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z),
    }


class ComparisonCsvLogger:
    def __init__(
        self,
        csv_path: Path,
        reference: SimulatorDescriptor,
        candidate: SimulatorDescriptor,
    ):
        self.csv_path = csv_path
        self.reference = reference
        self.candidate = candidate
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.handle,
            fieldnames=build_csv_fields(reference.csv_prefix, candidate.csv_prefix),
        )
        self.writer.writeheader()

    def _control_row(
        self,
        prefix: str,
        control: AppliedControlCommand,
    ) -> dict[str, str | int]:
        return {
            f"{prefix}_control_throttle": f"{control.throttle:.4f}",
            f"{prefix}_control_brake": f"{control.brake:.4f}",
            f"{prefix}_control_steer": f"{control.steer:.4f}",
            f"{prefix}_control_pedal": f"{control.pedal:.4f}",
            f"{prefix}_control_steering_angle_rad": f"{control.steering_angle_rad:.6f}",
            f"{prefix}_control_space": control.control_space,
            f"{prefix}_control_hand_brake": int(control.hand_brake),
            f"{prefix}_control_reverse": int(control.reverse),
        }

    def _state_row(
        self,
        prefix: str,
        state: VehicleState,
    ) -> dict[str, str]:
        return {
            f"{prefix}_x": f"{state.x:.6f}",
            f"{prefix}_y": f"{state.y:.6f}",
            f"{prefix}_z": f"{state.z:.6f}",
            f"{prefix}_yaw": f"{state.yaw:.6f}",
            f"{prefix}_speed": f"{state.speed:.6f}",
            f"{prefix}_acceleration": f"{state.acceleration:.6f}",
            f"{prefix}_vel_x": f"{state.vel_x:.6f}",
            f"{prefix}_vel_y": f"{state.vel_y:.6f}",
            f"{prefix}_vel_z": f"{state.vel_z:.6f}",
            f"{prefix}_ax": f"{state.ax:.6f}",
            f"{prefix}_ay": f"{state.ay:.6f}",
            f"{prefix}_az": f"{state.az:.6f}",
            f"{prefix}_wheel_angle": f"{state.wheel_angle:.6f}",
            f"{prefix}_wheel_rotation": f"{state.wheel_rotation:.6f}",
        }

    def write(
        self,
        step: int,
        input_control: InputControlCommand,
        control_source_space: str,
        control_mapping_strategy: str,
        reference_control: AppliedControlCommand,
        candidate_control: AppliedControlCommand,
        reference_state: VehicleState,
        candidate_state: VehicleState,
    ) -> dict[str, float]:
        diffs = compute_state_differences(reference_state, candidate_state)
        self.writer.writerow(
            {
                "step": step,
                "sim_time_s": f"{reference_state.timestamp_s:.3f}",
                "input_control_throttle": f"{input_control.throttle:.4f}",
                "input_control_brake": f"{input_control.brake:.4f}",
                "input_control_steer": f"{input_control.steer:.4f}",
                "input_control_hand_brake": int(input_control.hand_brake),
                "input_control_reverse": int(input_control.reverse),
                "control_source_space": control_source_space,
                "control_mapping_strategy": control_mapping_strategy,
                "reference_simulator_id": self.reference.simulator_id,
                "reference_label": self.reference.label,
                "reference_csv_prefix": self.reference.csv_prefix,
                "reference_backend": self.reference.backend_name,
                "candidate_simulator_id": self.candidate.simulator_id,
                "candidate_label": self.candidate.label,
                "candidate_csv_prefix": self.candidate.csv_prefix,
                "candidate_backend": self.candidate.backend_name,
                **self._control_row(self.reference.csv_prefix, reference_control),
                **self._control_row(self.candidate.csv_prefix, candidate_control),
                **self._state_row(self.reference.csv_prefix, reference_state),
                **self._state_row(self.candidate.csv_prefix, candidate_state),
                "diff_x": f"{diffs['diff_x']:.6f}",
                "diff_y": f"{diffs['diff_y']:.6f}",
                "diff_z": f"{diffs['diff_z']:.6f}",
                "diff_yaw": f"{diffs['diff_yaw']:.6f}",
                "diff_speed": f"{diffs['diff_speed']:.6f}",
                "diff_acceleration": f"{diffs['diff_acceleration']:.6f}",
                "diff_pos_2d": f"{diffs['diff_pos_2d']:.6f}",
                "diff_pos_3d": f"{diffs['diff_pos_3d']:.6f}",
            }
        )
        return diffs

    def close(self) -> None:
        self.handle.close()
