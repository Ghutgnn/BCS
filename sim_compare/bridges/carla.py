from __future__ import annotations

import math
from pathlib import Path

from sim_compare.control_spaces import CARLA_CONTROL_SPACE
from sim_compare.config import (
    CarlaInitializationConfig,
    CarlaOpenDriveGenerationConfig,
    CoordinateTransformConfig,
)
from sim_compare.models import AppliedControlCommand, InitialPose, VehicleState
from sim_compare.utils import clamp, normalize_yaw_rad


class CarlaBridge:
    simulator_name = "carla"
    control_space = CARLA_CONTROL_SPACE

    def __init__(
        self,
        host: str,
        port: int,
        timeout_s: float,
        traffic_manager_port: int,
        vehicle_filter: str,
        xodr_path: Path,
        dt_s: float,
        coordinate_transform: CoordinateTransformConfig,
        opendrive_config: CarlaOpenDriveGenerationConfig,
        initialization_config: CarlaInitializationConfig,
    ):
        import carla  # pylint: disable=import-outside-toplevel

        self.carla = carla
        self.vehicle_filter = vehicle_filter
        self.coordinate_transform = coordinate_transform
        self.initialization_config = initialization_config
        self.client = self.carla.Client(host, port)
        self.client.set_timeout(timeout_s)
        with xodr_path.open("r", encoding="utf-8") as handle:
            xodr_data = handle.read()

        self.world = self.client.generate_opendrive_world(
            xodr_data,
            self.carla.OpendriveGenerationParameters(
                vertex_distance=opendrive_config.vertex_distance,
                max_road_length=opendrive_config.max_road_length,
                wall_height=opendrive_config.wall_height,
                additional_width=opendrive_config.additional_width,
                smooth_junctions=True,
                enable_mesh_visibility=True,
            ),
        )
        self.original_settings = self.world.get_settings()
        self.traffic_manager = self.client.get_trafficmanager(traffic_manager_port)
        self.actor = None
        self._constant_velocity_bootstrap_active = False
        self._measurement_start_pose: InitialPose | None = None
        self._last_readiness_ticks = 0
        self._set_sync(dt_s)

    def _set_sync(self, dt_s: float) -> None:
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt_s
        self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(True)

    def _to_carla_y(self, esmini_y: float) -> float:
        return -esmini_y if self.coordinate_transform.invert_y else esmini_y

    def _to_esmini_y(self, carla_y: float) -> float:
        return -carla_y if self.coordinate_transform.invert_y else carla_y

    def _to_carla_yaw_deg(self, esmini_yaw_deg: float) -> float:
        return (
            self.coordinate_transform.yaw_sign * esmini_yaw_deg
            + self.coordinate_transform.yaw_offset_deg
        )

    def _to_esmini_yaw_rad(self, carla_yaw_deg: float) -> float:
        esmini_yaw_deg = (
            carla_yaw_deg - self.coordinate_transform.yaw_offset_deg
        ) / self.coordinate_transform.yaw_sign
        return normalize_yaw_rad(math.radians(esmini_yaw_deg))

    def _build_transform(self, pose: InitialPose):
        z = float(pose.carla_z)
        if self.initialization_config.snap_to_road_z:
            try:
                waypoint = self.world.get_map().get_waypoint(
                    self.carla.Location(
                        x=float(pose.x),
                        y=self._to_carla_y(float(pose.y)),
                        z=float(pose.carla_z),
                    ),
                    project_to_road=True,
                )
            except RuntimeError:
                waypoint = None
            if waypoint is not None:
                z = (
                    float(waypoint.transform.location.z)
                    + float(self.initialization_config.road_z_offset)
                )
        return self.carla.Transform(
            self.carla.Location(
                x=float(pose.x),
                y=self._to_carla_y(float(pose.y)),
                z=z,
            ),
            self.carla.Rotation(yaw=self._to_carla_yaw_deg(float(pose.yaw_deg))),
        )

    def _disable_constant_velocity_bootstrap(self) -> None:
        if self.actor is None or not self._constant_velocity_bootstrap_active:
            return
        disable = getattr(self.actor, "disable_constant_velocity", None)
        if callable(disable):
            disable()
        self._constant_velocity_bootstrap_active = False

    def _set_planar_target_speed(self, transform, speed_mps: float) -> None:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")
        forward = transform.get_forward_vector()
        velocity = self.carla.Vector3D(
            x=float(forward.x * speed_mps),
            y=float(forward.y * speed_mps),
            z=0.0,
        )
        enable = getattr(self.actor, "enable_constant_velocity", None)
        if (
            self.initialization_config.constant_velocity_bootstrap
            and callable(enable)
        ):
            enable(velocity)
            self.actor.set_target_velocity(velocity)
            self._constant_velocity_bootstrap_active = True
            materialization_ticks = max(
                1, int(self.initialization_config.constant_velocity_warmup_ticks)
            )
            for _ in range(materialization_ticks):
                self.world.tick()
        else:
            self.actor.set_target_velocity(velocity)
            self.world.tick()

    def _capture_measurement_start_pose(
        self,
        requested_pose: InitialPose,
    ) -> None:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")
        transform = self.actor.get_transform()
        yaw_deg = math.degrees(self._to_esmini_yaw_rad(transform.rotation.yaw))
        velocity = self.actor.get_velocity()
        speed_planar = math.sqrt(velocity.x**2 + velocity.y**2)
        self._measurement_start_pose = InitialPose(
            x=float(transform.location.x),
            y=float(self._to_esmini_y(transform.location.y)),
            yaw_deg=float(yaw_deg),
            speed=float(speed_planar),
            carla_z=float(transform.location.z),
        )

    def _is_ready_for_measurement(
        self,
        target_speed_mps: float,
    ) -> bool:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")
        velocity = self.actor.get_velocity()
        vertical_speed = abs(float(velocity.z))
        planar_speed = math.sqrt(float(velocity.x) ** 2 + float(velocity.y) ** 2)
        if (
            vertical_speed
            > float(self.initialization_config.readiness_vertical_speed_tolerance)
        ):
            return False
        if (
            abs(planar_speed - target_speed_mps)
            > float(self.initialization_config.readiness_planar_speed_tolerance)
        ):
            return False
        return True

    def _wait_until_ready_for_measurement(self, target_speed_mps: float) -> None:
        self._last_readiness_ticks = 0
        if not self.initialization_config.readiness_check_enabled:
            return
        required_consecutive = max(
            1, int(self.initialization_config.readiness_consecutive_ticks)
        )
        max_ticks = max(0, int(self.initialization_config.readiness_max_ticks))
        consecutive_ready = 0
        for tick_idx in range(max_ticks):
            self.world.tick()
            self._last_readiness_ticks = tick_idx + 1
            if self._is_ready_for_measurement(target_speed_mps):
                consecutive_ready += 1
                if consecutive_ready >= required_consecutive:
                    return
            else:
                consecutive_ready = 0

    def spawn_vehicle(self, pose: InitialPose) -> None:
        blueprints = self.world.get_blueprint_library().filter(self.vehicle_filter)
        if not blueprints:
            raise RuntimeError(
                f"No CARLA blueprint matching filter: {self.vehicle_filter}"
            )

        transform = self._build_transform(pose)
        self.actor = self.world.try_spawn_actor(blueprints[0], transform)
        if self.actor is None:
            raise RuntimeError(
                f"Failed to spawn CARLA actor at x={transform.location.x:.2f}, "
                f"y={transform.location.y:.2f}, z={transform.location.z:.2f}"
            )
        self.set_initial_pose(pose)

    def set_initial_pose(self, pose: InitialPose) -> None:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")
        self._disable_constant_velocity_bootstrap()
        transform = self._build_transform(pose)
        self.actor.set_simulate_physics(False)
        self.actor.set_transform(transform)
        self.actor.set_target_velocity(self.carla.Vector3D())
        self.actor.set_target_angular_velocity(self.carla.Vector3D())
        self.world.tick()
        self.actor.set_simulate_physics(True)
        self.actor.apply_control(
            self.carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
                steer=0.0,
                hand_brake=False,
                reverse=False,
            )
        )
        for _ in range(max(0, int(self.initialization_config.rest_settle_ticks))):
            self.world.tick()

        self.actor.apply_control(
            self.carla.VehicleControl(
                throttle=0.0,
                brake=0.0,
                steer=0.0,
                hand_brake=False,
                reverse=False,
            )
        )
        self.world.tick()
        if abs(pose.speed) > 0.0:
            self._set_planar_target_speed(transform, float(pose.speed))
        self._wait_until_ready_for_measurement(float(pose.speed))
        self._capture_measurement_start_pose(pose)

    def step(self, control: AppliedControlCommand, time_stamp_s: float) -> VehicleState:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")
        self._disable_constant_velocity_bootstrap()

        self.actor.apply_control(
            self.carla.VehicleControl(
                throttle=float(clamp(control.throttle, 0.0, 1.0)),
                brake=float(clamp(control.brake, 0.0, 1.0)),
                steer=float(clamp(control.steer, -1.0, 1.0)),
                hand_brake=bool(control.hand_brake),
                reverse=bool(control.reverse),
            )
        )
        self.world.tick()
        return self.get_state(time_stamp_s)

    def get_state(self, time_stamp_s: float) -> VehicleState:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")

        transform = self.actor.get_transform()
        velocity = self.actor.get_velocity()
        acceleration = self.actor.get_acceleration()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        planar_speed = math.sqrt(velocity.x**2 + velocity.y**2)
        acceleration_mag = math.sqrt(
            acceleration.x**2 + acceleration.y**2 + acceleration.z**2
        )
        planar_acceleration = math.sqrt(acceleration.x**2 + acceleration.y**2)
        return VehicleState(
            timestamp_s=float(time_stamp_s),
            x=float(transform.location.x),
            y=float(self._to_esmini_y(transform.location.y)),
            z=float(transform.location.z),
            yaw=float(self._to_esmini_yaw_rad(transform.rotation.yaw)),
            speed=float(speed),
            speed_planar=float(planar_speed),
            acceleration=float(acceleration_mag),
            acceleration_planar=float(planar_acceleration),
            vel_x=float(velocity.x),
            vel_y=float(self._to_esmini_y(velocity.y)),
            vel_z=float(velocity.z),
            ax=float(acceleration.x),
            ay=float(self._to_esmini_y(acceleration.y)),
            az=float(acceleration.z),
            wheel_angle=float("nan"),
            wheel_rotation=float("nan"),
        )

    def close(self) -> None:
        if self.actor is not None:
            self._disable_constant_velocity_bootstrap()
            self.actor.destroy()
            self.actor = None
        self._measurement_start_pose = None
        self.traffic_manager.set_synchronous_mode(False)
        self.world.apply_settings(self.original_settings)

    def get_measurement_start_pose(self) -> InitialPose | None:
        return self._measurement_start_pose

    def get_last_readiness_ticks(self) -> int:
        return self._last_readiness_ticks
