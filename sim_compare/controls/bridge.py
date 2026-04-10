from __future__ import annotations

import math

from sim_compare.models import CarlaControlCommand, EsminiControlCommand
from sim_compare.utils import clamp


def carla_control_to_esmini_simple_vehicle(
    control: CarlaControlCommand,
) -> EsminiControlCommand:
    throttle = control.throttle
    if control.reverse:
        throttle = -throttle
    throttle -= control.brake
    if control.hand_brake: 
        throttle = -1.0
    return EsminiControlCommand(
        backend="simple_vehicle_api",
        throttle=clamp(control.throttle, 0.0, 1.0),
        brake=clamp(control.brake, 0.0, 1.0),
        steer=clamp(control.steer, -1.0, 1.0) * -1.0,
        pedal=clamp(throttle, -1.0, 1.0),
        hand_brake=bool(control.hand_brake),
        reverse=bool(control.reverse),
    )


def carla_control_to_esmini_bcs_controller(
    control: CarlaControlCommand,
    max_steering_angle_rad: float,
) -> EsminiControlCommand:
    steer = clamp(control.steer, -1.0, 1.0) * -1.0
    brake = clamp(control.brake, 0.0, 1.0)
    if control.hand_brake:
        brake = 1.0
    return EsminiControlCommand(
        backend="bcs_controller",
        throttle=clamp(control.throttle, 0.0, 1.0),
        brake=brake,
        steer=steer,
        pedal=clamp(control.throttle - brake, -1.0, 1.0),
        steering_angle_rad=clamp(steer * max_steering_angle_rad, -math.pi / 2.0, math.pi / 2.0),
        hand_brake=bool(control.hand_brake),
        reverse=bool(control.reverse),
    )


# Backward-compatible alias for older code paths.
carla_control_to_esmini_udp_driver = carla_control_to_esmini_bcs_controller


def carla_control_to_esmini_control(
    control: CarlaControlCommand,
    backend: str,
    max_steering_angle_rad: float,
) -> EsminiControlCommand:
    if backend == "simple_vehicle_api":
        return carla_control_to_esmini_simple_vehicle(control)
    if backend in {"udp_driver_controller", "bcs_controller"}:
        return carla_control_to_esmini_bcs_controller(control, max_steering_angle_rad)
    raise ValueError(f"Unsupported esmini backend: {backend}")
