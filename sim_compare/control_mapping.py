from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from sim_compare.control_spaces import (
    CANONICAL_CONTROL_SPACE,
    CARLA_CONTROL_SPACE,
    ESMINI_BCS_CONTROL_SPACE,
    ESMINI_SIMPLE_CONTROL_SPACE,
    normalize_control_space,
)
from sim_compare.models import AppliedControlCommand, InputControlCommand
from sim_compare.utils import clamp


@dataclass
class ControlMappingContext:
    max_steering_angle_rad: float


def normalized_to_control_space(
    control_space: str,
    control: InputControlCommand,
    context: ControlMappingContext,
) -> AppliedControlCommand:
    space = normalize_control_space(control_space)
    if space in {CANONICAL_CONTROL_SPACE, CARLA_CONTROL_SPACE}:
        return AppliedControlCommand(
            control_space=space,
            throttle=clamp(control.throttle, 0.0, 1.0),
            brake=clamp(control.brake, 0.0, 1.0),
            steer=clamp(control.steer, -1.0, 1.0),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    if space == ESMINI_SIMPLE_CONTROL_SPACE:
        pedal = control.throttle
        if control.reverse:
            pedal = -pedal
        pedal -= control.brake
        if control.hand_brake:
            pedal = -1.0
        return AppliedControlCommand(
            control_space=space,
            throttle=clamp(control.throttle, 0.0, 1.0),
            brake=clamp(control.brake, 0.0, 1.0),
            steer=clamp(control.steer, -1.0, 1.0) * -1.0,
            pedal=clamp(pedal, -1.0, 1.0),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    if space == ESMINI_BCS_CONTROL_SPACE:
        steer = clamp(control.steer, -1.0, 1.0) * -1.0
        brake = clamp(control.brake, 0.0, 1.0)
        if control.hand_brake:
            brake = 1.0
        return AppliedControlCommand(
            control_space=space,
            throttle=clamp(control.throttle, 0.0, 1.0),
            brake=brake,
            steer=steer,
            pedal=clamp(control.throttle - brake, -1.0, 1.0),
            steering_angle_rad=clamp(
                steer * context.max_steering_angle_rad, -math.pi / 2.0, math.pi / 2.0
            ),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    raise ValueError(f"Unsupported target control space: {control_space}")


def control_space_to_normalized(
    control: AppliedControlCommand,
    context: ControlMappingContext,
) -> InputControlCommand:
    space = normalize_control_space(control.control_space)
    if space in {CANONICAL_CONTROL_SPACE, CARLA_CONTROL_SPACE}:
        return InputControlCommand(
            throttle=clamp(control.throttle, 0.0, 1.0),
            brake=clamp(control.brake, 0.0, 1.0),
            steer=clamp(control.steer, -1.0, 1.0),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    if space == ESMINI_SIMPLE_CONTROL_SPACE:
        if control.throttle > 0.0 or control.brake > 0.0:
            throttle = clamp(control.throttle, 0.0, 1.0)
            brake = clamp(control.brake, 0.0, 1.0)
        else:
            throttle = clamp(max(control.pedal, 0.0), 0.0, 1.0)
            brake = clamp(max(-control.pedal, 0.0), 0.0, 1.0)
        return InputControlCommand(
            throttle=throttle,
            brake=brake,
            steer=clamp(-control.steer, -1.0, 1.0),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    if space == ESMINI_BCS_CONTROL_SPACE:
        steer = control.steer
        if abs(steer) < 1e-9 and abs(control.steering_angle_rad) > 1e-9:
            if abs(context.max_steering_angle_rad) < 1e-9:
                steer = 0.0
            else:
                steer = control.steering_angle_rad / context.max_steering_angle_rad
        return InputControlCommand(
            throttle=clamp(control.throttle, 0.0, 1.0),
            brake=clamp(control.brake, 0.0, 1.0),
            steer=clamp(-steer, -1.0, 1.0),
            hand_brake=bool(control.hand_brake),
            reverse=bool(control.reverse),
        )

    raise ValueError(f"Unsupported source control space: {control.control_space}")


class ControlMapper(Protocol):
    name: str

    def map(
        self,
        source_control: AppliedControlCommand,
        target_control_space: str,
        context: ControlMappingContext,
    ) -> AppliedControlCommand:
        ...


class SemanticRoundtripMapper:
    name = "semantic_roundtrip"

    def map(
        self,
        source_control: AppliedControlCommand,
        target_control_space: str,
        context: ControlMappingContext,
    ) -> AppliedControlCommand:
        normalized = control_space_to_normalized(source_control, context)
        return normalized_to_control_space(target_control_space, normalized, context)


class DirectNumericMapper:
    name = "direct_numeric"

    def map(
        self,
        source_control: AppliedControlCommand,
        target_control_space: str,
        context: ControlMappingContext,
    ) -> AppliedControlCommand:
        source_space = normalize_control_space(source_control.control_space)
        target_space = normalize_control_space(target_control_space)

        if target_space == CARLA_CONTROL_SPACE:
            if source_space == ESMINI_SIMPLE_CONTROL_SPACE:
                return AppliedControlCommand(
                    control_space=CARLA_CONTROL_SPACE,
                    throttle=clamp(max(source_control.pedal, 0.0), 0.0, 1.0),
                    brake=clamp(max(-source_control.pedal, 0.0), 0.0, 1.0),
                    steer=clamp(source_control.steer, -1.0, 1.0),
                    hand_brake=bool(source_control.hand_brake),
                    reverse=bool(source_control.reverse),
                )
            if source_space == ESMINI_BCS_CONTROL_SPACE:
                steer = source_control.steer
                if abs(steer) < 1e-9 and abs(context.max_steering_angle_rad) > 1e-9:
                    steer = source_control.steering_angle_rad / context.max_steering_angle_rad
                return AppliedControlCommand(
                    control_space=CARLA_CONTROL_SPACE,
                    throttle=clamp(source_control.throttle, 0.0, 1.0),
                    brake=clamp(source_control.brake, 0.0, 1.0),
                    steer=clamp(steer, -1.0, 1.0),
                    hand_brake=bool(source_control.hand_brake),
                    reverse=bool(source_control.reverse),
                )

        if target_space == ESMINI_SIMPLE_CONTROL_SPACE and source_space == CARLA_CONTROL_SPACE:
            return AppliedControlCommand(
                control_space=ESMINI_SIMPLE_CONTROL_SPACE,
                throttle=clamp(source_control.throttle, 0.0, 1.0),
                brake=clamp(source_control.brake, 0.0, 1.0),
                steer=clamp(source_control.steer, -1.0, 1.0),
                pedal=clamp(source_control.throttle - source_control.brake, -1.0, 1.0),
                hand_brake=bool(source_control.hand_brake),
                reverse=bool(source_control.reverse),
            )

        if target_space == ESMINI_BCS_CONTROL_SPACE and source_space == CARLA_CONTROL_SPACE:
            return AppliedControlCommand(
                control_space=ESMINI_BCS_CONTROL_SPACE,
                throttle=clamp(source_control.throttle, 0.0, 1.0),
                brake=clamp(source_control.brake, 0.0, 1.0),
                steer=clamp(source_control.steer, -1.0, 1.0),
                steering_angle_rad=clamp(
                    source_control.steer * context.max_steering_angle_rad,
                    -math.pi / 2.0,
                    math.pi / 2.0,
                ),
                hand_brake=bool(source_control.hand_brake),
                reverse=bool(source_control.reverse),
            )

        normalized = control_space_to_normalized(source_control, context)
        return normalized_to_control_space(target_space, normalized, context)


CONTROL_MAPPERS: dict[str, ControlMapper] = {
    SemanticRoundtripMapper.name: SemanticRoundtripMapper(),
    DirectNumericMapper.name: DirectNumericMapper(),
}


def get_control_mapper(name: str) -> ControlMapper:
    key = name.strip().lower()
    mapper = CONTROL_MAPPERS.get(key)
    if mapper is None:
        raise ValueError(f"Unsupported control mapper: {name}")
    return mapper
