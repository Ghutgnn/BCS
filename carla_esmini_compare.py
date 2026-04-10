#!/usr/bin/env python3
"""Thin CLI wrapper for the CARLA/esmini comparison framework."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sim_compare.config import (
    CarlaOpenDriveGenerationConfig,
    CoordinateTransformConfig,
    EsminiBackendConfig,
    EsminiOptions,
    ExperimentConfig,
    SimulatorInstanceConfig,
)
from sim_compare.control_mapping import CONTROL_MAPPERS
from sim_compare.control_spaces import SUPPORTED_CONTROL_SPACES, normalize_control_space
from sim_compare.esmini_backend import (
    SUPPORTED_ESMINI_BACKENDS,
    normalize_esmini_backend_mode,
)
from sim_compare.models import InitialPose
from sim_compare.runner import ExperimentRunner
from sim_compare.simulators import SUPPORTED_SIMULATORS, normalize_simulator_id
from sim_compare.utils import parse_resolution


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare two simulator adapters on the same map, pose, and control."
    )
    parser.add_argument(
        "map_name", help="Map name under maps/: [map_name].xosc and [map_name].xodr"
    )
    parser.add_argument(
        "--esmini-home",
        type=Path,
        default=Path("/home/hcis-s05/ysws/BCS/esmini"),
        help="esmini installation root",
    )
    parser.add_argument(
        "--esmini-path",
        action="append",
        type=Path,
        default=[],
        help="Extra esmini search path (can repeat)",
    )
    parser.add_argument(
        "--ego-index", type=int, default=0, help="esmini ego object index"
    )
    parser.add_argument("--esmini-viewer", type=int, default=1, choices=[0, 1])
    parser.add_argument(
        "--esmini-disable-controller",
        type=int,
        default=1,
        choices=[0, 1],
        help="Deprecated. The selected --esmini-backend decides whether esmini controllers are enabled.",
    )
    parser.add_argument(
        "--esmini-backend",
        choices=sorted(SUPPORTED_ESMINI_BACKENDS),
        default="simple_vehicle_api",
        help="simple_vehicle_api keeps the old SE_SimpleVehicle path; "
        "bcs_controller uses the custom esmini BCS controller over UDP; "
        "udp_driver_controller is kept as a backward-compatible alias",
    )
    parser.add_argument("--esmini-udp-base-port", type=int, default=49950)
    parser.add_argument(
        "--esmini-udp-exec-mode",
        choices=["synchronous", "asynchronous"],
        default="synchronous",
    )
    parser.add_argument(
        "--simple-vehicle-config",
        type=Path,
        default=None,
        help="TOML file describing esmini simple vehicle parameters",
    )
    parser.add_argument(
        "--reference-simulator",
        choices=sorted(SUPPORTED_SIMULATORS),
        default="carla",
        help="Reference simulator used for diff sign and CSV prefix ordering",
    )
    parser.add_argument(
        "--candidate-simulator",
        choices=sorted(SUPPORTED_SIMULATORS),
        default="esmini",
        help="Candidate simulator compared against the reference simulator",
    )
    parser.add_argument(
        "--reference-label",
        default="carla",
        help="CSV/display label for the reference simulator",
    )
    parser.add_argument(
        "--candidate-label",
        default="esmini",
        help="CSV/display label for the candidate simulator",
    )

    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--carla-timeout", type=float, default=20.0)
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument("--carla-vehicle-filter", default="vehicle.tesla.model3")

    parser.add_argument("--init-x", type=float, required=True)
    parser.add_argument("--init-y", type=float, required=True)
    parser.add_argument("--init-yaw-deg", type=float, default=0.0)
    parser.add_argument("--init-speed", type=float, default=0.0)
    parser.add_argument("--carla-init-z", type=float, default=0.5)

    parser.add_argument("--input-mode", choices=["series", "keyboard"], required=True)
    parser.add_argument("--control-csv", type=Path, default=None)
    parser.add_argument("--hold-last-control", action="store_true")
    parser.add_argument(
        "--control-source-space",
        choices=sorted(SUPPORTED_CONTROL_SPACES),
        default="canonical.actuation",
        help="Interpret the external input as native to this control space before mapping to each simulator",
    )
    parser.add_argument(
        "--control-mapping-strategy",
        choices=sorted(CONTROL_MAPPERS.keys()),
        default="semantic_roundtrip",
        help="Strategy used to map control between simulator control spaces",
    )

    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--max-steps", type=int, default=1000000)
    parser.add_argument("--csv-out", type=Path, default=None)
    parser.add_argument("--plot-out", type=Path, default=None)
    parser.add_argument("--print-every", type=int, default=20)

    parser.add_argument(
        "--render-camera",
        "--render-carla",
        action="store_true",
        dest="render_camera",
        help="Render the first simulator that exposes a camera/display target (currently CARLA)",
    )
    parser.add_argument("--res", default="1920x1080")
    parser.add_argument("--gamma", type=float, default=2.2)

    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--yaw-sign", type=float, default=1.0, choices=[-1.0, 1.0])
    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)

    parser.add_argument("--vertex-distance", type=float, default=2.0)
    parser.add_argument("--max-road-length", type=float, default=500.0)
    parser.add_argument("--wall-height", type=float, default=1.0)
    parser.add_argument("--additional-width", type=float, default=0.6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    project_root = Path(__file__).resolve().parent
    resolution = parse_resolution(args.res)
    csv_out = args.csv_out or (project_root / f"comparison_{args.map_name}.csv")
    plot_out = args.plot_out or csv_out.with_suffix(".svg")
    simple_vehicle_config = None
    if args.simple_vehicle_config is not None:
        simple_vehicle_config = args.simple_vehicle_config
        if not simple_vehicle_config.is_absolute():
            simple_vehicle_config = (project_root / simple_vehicle_config).resolve()
        else:
            simple_vehicle_config = simple_vehicle_config.resolve()
    normalized_esmini_backend = normalize_esmini_backend_mode(args.esmini_backend)
    normalized_control_source_space = normalize_control_space(args.control_source_space)
    reference_simulator = normalize_simulator_id(args.reference_simulator)
    candidate_simulator = normalize_simulator_id(args.candidate_simulator)

    config = ExperimentConfig(
        project_root=project_root,
        map_name=args.map_name,
        esmini_home=args.esmini_home.resolve(),
        extra_esmini_paths=[path.resolve() for path in args.esmini_path],
        ego_index=args.ego_index,
        initial_pose=InitialPose(
            x=float(args.init_x),
            y=float(args.init_y),
            yaw_deg=float(args.init_yaw_deg),
            speed=float(args.init_speed),
            carla_z=float(args.carla_init_z),
        ),
        input_mode=args.input_mode,
        control_csv=args.control_csv.resolve() if args.control_csv else None,
        hold_last_control=bool(args.hold_last_control),
        control_source_space=normalized_control_source_space,
        control_mapping_strategy=args.control_mapping_strategy,
        dt=float(args.dt),
        max_steps=int(args.max_steps),
        csv_out=csv_out.resolve(),
        plot_out=plot_out.resolve(),
        print_every=int(args.print_every),
        render_camera=bool(args.render_camera),
        resolution=resolution,
        gamma=float(args.gamma),
        reference_simulator=SimulatorInstanceConfig(
            simulator_id=reference_simulator,
            label=args.reference_label,
        ),
        candidate_simulator=SimulatorInstanceConfig(
            simulator_id=candidate_simulator,
            label=args.candidate_label,
        ),
        coordinate_transform=CoordinateTransformConfig(
            invert_y=bool(args.invert_y),
            yaw_sign=float(args.yaw_sign),
            yaw_offset_deg=float(args.yaw_offset_deg),
        ),
        carla_opendrive=CarlaOpenDriveGenerationConfig(
            vertex_distance=float(args.vertex_distance),
            max_road_length=float(args.max_road_length),
            wall_height=float(args.wall_height),
            additional_width=float(args.additional_width),
        ),
        esmini_options=EsminiOptions(
            use_viewer=int(args.esmini_viewer),
            disable_controller=(
                1 if normalized_esmini_backend == "simple_vehicle_api" else 0
            ),
            window=(60, 60, resolution[0], resolution[1]),
            disable_stdout=False,
            log_file_path=csv_out.with_suffix(".esmini.log"),
            dat_file_path=None,
        ),
        esmini_backend=EsminiBackendConfig(
            mode=normalized_esmini_backend,
            udp_base_port=int(args.esmini_udp_base_port),
            udp_exec_mode=args.esmini_udp_exec_mode,
            simple_vehicle_config_path=simple_vehicle_config,
        ),
        carla_host=args.carla_host,
        carla_port=int(args.carla_port),
        carla_timeout=float(args.carla_timeout),
        traffic_manager_port=int(args.traffic_manager_port),
        carla_vehicle_filter=args.carla_vehicle_filter,
    )
    return ExperimentRunner(config).run()


if __name__ == "__main__":
    sys.exit(main())
