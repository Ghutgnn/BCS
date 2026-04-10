from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CarlaControlCommand:
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    hand_brake: bool = False
    reverse: bool = False


@dataclass
class EsminiControlCommand:
    backend: str = "simple_vehicle_api"
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    pedal: float = 0.0
    steering_angle_rad: float = 0.0
    hand_brake: bool = False
    reverse: bool = False


@dataclass
class InitialPose:
    x: float
    y: float
    yaw_deg: float
    speed: float
    carla_z: float = 0.5


@dataclass
class VehicleState:
    timestamp_s: float
    x: float
    y: float
    z: float
    yaw: float
    speed: float
    acceleration: float
    vel_x: float
    vel_y: float
    vel_z: float
    ax: float
    ay: float
    az: float
    wheel_angle: float
    wheel_rotation: float
