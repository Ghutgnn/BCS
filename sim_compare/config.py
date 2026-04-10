from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .models import InitialPose


@dataclass
class CoordinateTransformConfig:
    invert_y: bool = False
    yaw_sign: float = 1.0
    yaw_offset_deg: float = 0.0


@dataclass
class CarlaOpenDriveGenerationConfig:
    vertex_distance: float = 2.0
    max_road_length: float = 500.0
    wall_height: float = 1.0
    additional_width: float = 0.6


@dataclass
class EsminiOptions:
    use_viewer: int = 1
    disable_controller: int = 1
    threads: int = 0
    record: int = 0
    seed: int = 1234
    disable_stdout: bool = True
    window: tuple[int, int, int, int] | None = (60, 60, 1280, 720)
    log_file_path: Path | None = None
    dat_file_path: Path | None = None


@dataclass
class EsminiBackendConfig:
    mode: str = "simple_vehicle_api"
    udp_base_port: int = 49950
    udp_exec_mode: str = "synchronous"
    simple_vehicle_config_path: Path | None = None


@dataclass
class ExperimentConfig:
    project_root: Path
    map_name: str
    esmini_home: Path
    extra_esmini_paths: list[Path]
    ego_index: int
    initial_pose: InitialPose
    input_mode: str
    control_csv: Path | None
    hold_last_control: bool
    dt: float
    max_steps: int
    csv_out: Path
    plot_out: Path | None
    print_every: int
    render_carla: bool
    resolution: tuple[int, int]
    gamma: float
    coordinate_transform: CoordinateTransformConfig = field(
        default_factory=CoordinateTransformConfig
    )
    carla_opendrive: CarlaOpenDriveGenerationConfig = field(
        default_factory=CarlaOpenDriveGenerationConfig
    )
    esmini_options: EsminiOptions = field(default_factory=EsminiOptions)
    esmini_backend: EsminiBackendConfig = field(default_factory=EsminiBackendConfig)
    carla_host: str = "127.0.0.1"
    carla_port: int = 2000
    carla_timeout: float = 20.0
    traffic_manager_port: int = 8000
    carla_vehicle_filter: str = "vehicle.tesla.model3"
