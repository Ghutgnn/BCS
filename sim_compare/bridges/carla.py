from __future__ import annotations

import math
from pathlib import Path

from sim_compare.config import CarlaOpenDriveGenerationConfig, CoordinateTransformConfig
from sim_compare.models import CarlaControlCommand, InitialPose, VehicleState
from sim_compare.utils import clamp, normalize_yaw_rad


class CarlaBridge:
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
    ):
        import carla  # pylint: disable=import-outside-toplevel

        self.carla = carla
        self.vehicle_filter = vehicle_filter
        self.coordinate_transform = coordinate_transform
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

    def spawn_vehicle(self, pose: InitialPose) -> None:
        blueprints = self.world.get_blueprint_library().filter(self.vehicle_filter)
        if not blueprints:
            raise RuntimeError(
                f"No CARLA blueprint matching filter: {self.vehicle_filter}"
            )

        transform = self.carla.Transform(
            self.carla.Location(
                x=float(pose.x),
                y=self._to_carla_y(float(pose.y)),
                z=float(pose.carla_z),
            ),
            self.carla.Rotation(yaw=self._to_carla_yaw_deg(float(pose.yaw_deg))),
        )
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
        transform = self.carla.Transform(
            self.carla.Location(
                x=float(pose.x),
                y=self._to_carla_y(float(pose.y)),
                z=float(pose.carla_z),
            ),
            self.carla.Rotation(yaw=self._to_carla_yaw_deg(float(pose.yaw_deg))),
        )
        self.actor.set_transform(transform)
        self.actor.set_target_velocity(self.carla.Vector3D())
        self.actor.set_target_angular_velocity(self.carla.Vector3D())
        if abs(pose.speed) > 0.0:
            forward = transform.get_forward_vector()
            self.actor.set_target_velocity(
                self.carla.Vector3D(
                    x=forward.x * pose.speed,
                    y=forward.y * pose.speed,
                    z=0.0,
                )
            )
        self.world.tick()

    def step(self, control: CarlaControlCommand, time_stamp_s: float) -> VehicleState:
        if self.actor is None:
            raise RuntimeError("CARLA actor not initialized")

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
        acceleration_mag = math.sqrt(
            acceleration.x**2 + acceleration.y**2 + acceleration.z**2
        )
        return VehicleState(
            timestamp_s=float(time_stamp_s),
            x=float(transform.location.x),
            y=float(self._to_esmini_y(transform.location.y)),
            z=float(transform.location.z),
            yaw=float(self._to_esmini_yaw_rad(transform.rotation.yaw)),
            speed=float(speed),
            acceleration=float(acceleration_mag),
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
            self.actor.destroy()
            self.actor = None
        self.traffic_manager.set_synchronous_mode(False)
        self.world.apply_settings(self.original_settings)
