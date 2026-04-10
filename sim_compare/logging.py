from __future__ import annotations

import csv
import math
from pathlib import Path

from sim_compare.models import (
    CarlaControlCommand,
    EsminiControlCommand,
    VehicleState,
)
from sim_compare.utils import normalize_yaw_rad


CSV_FIELDS = [
    "step",
    "sim_time_s",
    "carla_control_throttle",
    "carla_control_brake",
    "carla_control_steer",
    "carla_control_hand_brake",
    "carla_control_reverse",
    "esmini_control_throttle",
    "esmini_control_brake",
    "esmini_control_steer",
    "esmini_control_pedal",
    "esmini_control_steering_angle_rad",
    "esmini_control_backend",
    "esmini_control_hand_brake",
    "esmini_control_reverse",
    "carla_x",
    "carla_y",
    "carla_z",
    "carla_yaw",
    "carla_speed",
    "carla_acceleration",
    "carla_vel_x",
    "carla_vel_y",
    "carla_vel_z",
    "carla_ax",
    "carla_ay",
    "carla_az",
    "esmini_x",
    "esmini_y",
    "esmini_z",
    "esmini_yaw",
    "esmini_speed",
    "esmini_acceleration",
    "esmini_vel_x",
    "esmini_vel_y",
    "esmini_vel_z",
    "esmini_ax",
    "esmini_ay",
    "esmini_az",
    "esmini_wheel_angle",
    "esmini_wheel_rotation",
    "diff_x",
    "diff_y",
    "diff_z",
    "diff_yaw",
    "diff_speed",
    "diff_acceleration",
    "diff_pos_2d",
    "diff_pos_3d",
]


def compute_state_differences(
    carla_state: VehicleState,
    esmini_state: VehicleState,
) -> dict[str, float]:
    diff_x = carla_state.x - esmini_state.x
    diff_y = carla_state.y - esmini_state.y
    diff_z = carla_state.z - esmini_state.z
    return {
        "diff_x": diff_x,
        "diff_y": diff_y,
        "diff_z": diff_z,
        "diff_yaw": normalize_yaw_rad(carla_state.yaw - esmini_state.yaw),
        "diff_speed": carla_state.speed - esmini_state.speed,
        "diff_acceleration": carla_state.acceleration - esmini_state.acceleration,
        "diff_pos_2d": math.sqrt(diff_x * diff_x + diff_y * diff_y),
        "diff_pos_3d": math.sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z),
    }


class ComparisonCsvLogger:
    def __init__(self, csv_path: Path):
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

    def write(
        self,
        step: int,
        carla_control: CarlaControlCommand,
        esmini_control: EsminiControlCommand,
        carla_state: VehicleState,
        esmini_state: VehicleState,
    ) -> dict[str, float]:
        diffs = compute_state_differences(carla_state, esmini_state)
        self.writer.writerow(
            {
                "step": step,
                "sim_time_s": f"{carla_state.timestamp_s:.3f}",
                "carla_control_throttle": f"{carla_control.throttle:.4f}",
                "carla_control_brake": f"{carla_control.brake:.4f}",
                "carla_control_steer": f"{carla_control.steer:.4f}",
                "carla_control_hand_brake": int(carla_control.hand_brake),
                "carla_control_reverse": int(carla_control.reverse),
                "esmini_control_throttle": f"{esmini_control.throttle:.4f}",
                "esmini_control_brake": f"{esmini_control.brake:.4f}",
                "esmini_control_steer": f"{esmini_control.steer:.4f}",
                "esmini_control_pedal": f"{esmini_control.pedal:.4f}",
                "esmini_control_steering_angle_rad": f"{esmini_control.steering_angle_rad:.6f}",
                "esmini_control_backend": esmini_control.backend,
                "esmini_control_hand_brake": int(esmini_control.hand_brake),
                "esmini_control_reverse": int(esmini_control.reverse),
                "carla_x": f"{carla_state.x:.6f}",
                "carla_y": f"{carla_state.y:.6f}",
                "carla_z": f"{carla_state.z:.6f}",
                "carla_yaw": f"{carla_state.yaw:.6f}",
                "carla_speed": f"{carla_state.speed:.6f}",
                "carla_acceleration": f"{carla_state.acceleration:.6f}",
                "carla_vel_x": f"{carla_state.vel_x:.6f}",
                "carla_vel_y": f"{carla_state.vel_y:.6f}",
                "carla_vel_z": f"{carla_state.vel_z:.6f}",
                "carla_ax": f"{carla_state.ax:.6f}",
                "carla_ay": f"{carla_state.ay:.6f}",
                "carla_az": f"{carla_state.az:.6f}",
                "esmini_x": f"{esmini_state.x:.6f}",
                "esmini_y": f"{esmini_state.y:.6f}",
                "esmini_z": f"{esmini_state.z:.6f}",
                "esmini_yaw": f"{esmini_state.yaw:.6f}",
                "esmini_speed": f"{esmini_state.speed:.6f}",
                "esmini_acceleration": f"{esmini_state.acceleration:.6f}",
                "esmini_vel_x": f"{esmini_state.vel_x:.6f}",
                "esmini_vel_y": f"{esmini_state.vel_y:.6f}",
                "esmini_vel_z": f"{esmini_state.vel_z:.6f}",
                "esmini_ax": f"{esmini_state.ax:.6f}",
                "esmini_ay": f"{esmini_state.ay:.6f}",
                "esmini_az": f"{esmini_state.az:.6f}",
                "esmini_wheel_angle": f"{esmini_state.wheel_angle:.6f}",
                "esmini_wheel_rotation": f"{esmini_state.wheel_rotation:.6f}",
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
