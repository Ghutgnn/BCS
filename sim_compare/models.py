from __future__ import annotations

from dataclasses import dataclass

from sim_compare.control_spaces import CARLA_CONTROL_SPACE


@dataclass
class InputControlCommand:
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    hand_brake: bool = False
    reverse: bool = False


@dataclass
class AppliedControlCommand:
    control_space: str = CARLA_CONTROL_SPACE
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


# Backward-compatible aliases while the rest of the codebase migrates.
CanonicalControlCommand = InputControlCommand
CarlaControlCommand = AppliedControlCommand
EsminiControlCommand = AppliedControlCommand
