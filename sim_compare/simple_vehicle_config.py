from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tomllib


@dataclass
class SimpleVehicleConfig:
    name: str = "simple_vehicle"
    length_m: float | None = None
    max_speed_mps: float | None = None
    max_acceleration_mps2: float | None = None
    max_deceleration_mps2: float | None = None
    engine_brake_factor: float | None = None
    steering_scale: float | None = None
    steering_return_factor: float | None = None
    steering_rate: float | None = None
    throttle_disabled: bool = False
    steering_disabled: bool = False
    source_path: Path | None = None


def _optional_float(data: dict, key: str) -> float | None:
    value = data.get(key)
    if value is None:
        return None
    return float(value)


def load_simple_vehicle_config(path: Path) -> SimpleVehicleConfig:
    with path.open("rb") as handle:
        raw = tomllib.load(handle)

    vehicle_data = raw.get("vehicle", raw)
    config = SimpleVehicleConfig(
        name=str(vehicle_data.get("name", path.stem)),
        length_m=_optional_float(vehicle_data, "length_m"),
        max_speed_mps=_optional_float(vehicle_data, "max_speed_mps"),
        max_acceleration_mps2=_optional_float(vehicle_data, "max_acceleration_mps2"),
        max_deceleration_mps2=_optional_float(vehicle_data, "max_deceleration_mps2"),
        engine_brake_factor=_optional_float(vehicle_data, "engine_brake_factor"),
        steering_scale=_optional_float(vehicle_data, "steering_scale"),
        steering_return_factor=_optional_float(vehicle_data, "steering_return_factor"),
        steering_rate=_optional_float(vehicle_data, "steering_rate"),
        throttle_disabled=bool(vehicle_data.get("throttle_disabled", False)),
        steering_disabled=bool(vehicle_data.get("steering_disabled", False)),
        source_path=path,
    )
    _validate_simple_vehicle_config(config)
    return config


def _validate_non_negative(name: str, value: float | None) -> None:
    if value is not None and value < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value}")


def _validate_range(name: str, value: float | None, lo: float, hi: float) -> None:
    if value is not None and not (lo <= value <= hi):
        raise ValueError(f"{name} must be in [{lo}, {hi}], got {value}")


def _validate_simple_vehicle_config(config: SimpleVehicleConfig) -> None:
    _validate_non_negative("length_m", config.length_m)
    _validate_non_negative("max_speed_mps", config.max_speed_mps)
    _validate_non_negative("max_acceleration_mps2", config.max_acceleration_mps2)
    _validate_non_negative("max_deceleration_mps2", config.max_deceleration_mps2)
    _validate_non_negative("steering_rate", config.steering_rate)
    _validate_non_negative("steering_return_factor", config.steering_return_factor)
    _validate_range("engine_brake_factor", config.engine_brake_factor, 0.0, 1.0)
    _validate_non_negative("steering_scale", config.steering_scale)
