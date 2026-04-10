from __future__ import annotations

import ctypes as ct
import math

from sim_compare.bridges.esmini_runtime import EsminiRuntime, SESimpleVehicleState
from sim_compare.models import EsminiControlCommand, InitialPose, VehicleState
from sim_compare.simple_vehicle_config import SimpleVehicleConfig


class EsminiSimpleVehicleBridge:
    backend_name = "simple_vehicle_api"

    def __init__(
        self,
        runtime: EsminiRuntime,
        xosc_path,
        vehicle_config: SimpleVehicleConfig | None = None,
    ):
        self.runtime = runtime
        self.xosc_path = xosc_path
        self.vehicle_config = vehicle_config
        self.initial_pose: InitialPose | None = None
        self.sv_handle = None
        self.sv_state = SESimpleVehicleState()

    def start(self) -> None:
        self.runtime.start(self.xosc_path, disable_controller=1)
        if self.initial_pose is not None:
            self._apply_initial_pose(self.initial_pose)

    def set_initial_pose(self, pose: InitialPose) -> None:
        self.initial_pose = pose
        if self.runtime.obj_id >= 0:
            self._apply_initial_pose(pose)

    def _apply_initial_pose(self, pose: InitialPose) -> None:
        if self.sv_handle is not None:
            self.runtime.se.SE_SimpleVehicleDelete(self.sv_handle)
            self.sv_handle = None

        yaw_rad = math.radians(pose.yaw_deg)
        vehicle_length = (
            float(self.vehicle_config.length_m)
            if self.vehicle_config is not None and self.vehicle_config.length_m is not None
            else float(self.runtime.ego_length)
        )
        self.sv_handle = self.runtime.se.SE_SimpleVehicleCreate(
            float(pose.x),
            float(pose.y),
            float(yaw_rad),
            vehicle_length,
            float(pose.speed),
        )
        self._apply_vehicle_config(float(pose.speed))
        self.runtime.se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.sv_state))
        self.runtime.report_initial_pose(
            x=float(pose.x),
            y=float(pose.y),
            yaw_rad=float(yaw_rad),
            speed=float(pose.speed),
        )

    def _apply_vehicle_config(self, initial_speed_mps: float) -> None:
        if self.sv_handle is None or self.vehicle_config is None:
            return

        se = self.runtime.se
        config = self.vehicle_config
        se.SE_SimpleVehicleSetThrottleDisabled(
            self.sv_handle, bool(config.throttle_disabled)
        )
        se.SE_SimpleVehicleSetSteeringDisabled(
            self.sv_handle, bool(config.steering_disabled)
        )
        se.SE_SimpleVehicleSetSpeed(self.sv_handle, float(initial_speed_mps))

        if config.max_speed_mps is not None:
            # esmini simple vehicle API expects km/h for this setter.
            se.SE_SimpleVehicleSetMaxSpeed(
                self.sv_handle, float(config.max_speed_mps * 3.6)
            )
        if config.max_acceleration_mps2 is not None:
            se.SE_SimpleVehicleSetMaxAcceleration(
                self.sv_handle, float(config.max_acceleration_mps2)
            )
        if config.max_deceleration_mps2 is not None:
            se.SE_SimpleVehicleSetMaxDeceleration(
                self.sv_handle, float(config.max_deceleration_mps2)
            )
        if config.engine_brake_factor is not None:
            se.SE_SimpleVehicleSetEngineBrakeFactor(
                self.sv_handle, float(config.engine_brake_factor)
            )
        if config.steering_scale is not None:
            se.SE_SimpleVehicleSteeringScale(
                self.sv_handle, float(config.steering_scale)
            )
        if config.steering_return_factor is not None:
            se.SE_SimpleVehicleSteeringReturnFactor(
                self.sv_handle, float(config.steering_return_factor)
            )
        if config.steering_rate is not None:
            se.SE_SimpleVehicleSteeringRate(
                self.sv_handle, float(config.steering_rate)
            )

    def step(
        self,
        control: EsminiControlCommand,
        dt_s: float,
        timestamp_s: float,
    ) -> VehicleState:
        if self.sv_handle is None:
            raise RuntimeError("esmini simple vehicle is not initialized")

        self.runtime.se.SE_SimpleVehicleControlAnalog(
            self.sv_handle,
            float(dt_s),
            float(control.pedal),
            float(control.steer),
        )
        self.runtime.se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.sv_state))
        self.runtime.se.SE_ReportObjectPosXYH(
            self.runtime.obj_id,
            self.sv_state.x,
            self.sv_state.y,
            self.sv_state.h,
        )
        self.runtime.se.SE_ReportObjectWheelStatus(
            self.runtime.obj_id,
            self.sv_state.wheel_rotation,
            self.sv_state.wheel_angle,
        )
        self.runtime.se.SE_ReportObjectSpeed(self.runtime.obj_id, self.sv_state.speed)
        ret = self.runtime.se.SE_StepDT(float(dt_s))
        if ret != 0:
            raise RuntimeError(f"SE_StepDT failed in simple vehicle backend, ret={ret}")
        return self.runtime.get_ego_state(timestamp_s)

    def get_state(self, timestamp_s: float) -> VehicleState:
        return self.runtime.get_ego_state(timestamp_s)

    def should_quit(self) -> bool:
        return self.runtime.should_quit()

    def close(self) -> None:
        if self.sv_handle is not None:
            self.runtime.se.SE_SimpleVehicleDelete(self.sv_handle)
            self.sv_handle = None
        self.runtime.close()
