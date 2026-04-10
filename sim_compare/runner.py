from __future__ import annotations

import sys

from sim_compare.bridges.carla import CarlaBridge
from sim_compare.bridges.esmini import create_esmini_bridge
from sim_compare.bridges.esmini_scenario import read_front_axle_max_steering_rad
from sim_compare.config import ExperimentConfig
from sim_compare.controls.bridge import carla_control_to_esmini_control
from sim_compare.controls.sources import (
    KeyboardControlSource,
    SeriesControlSource,
)
from sim_compare.logging import ComparisonCsvLogger
from sim_compare.maps import build_esmini_search_paths, resolve_map_paths
from sim_compare.models import CarlaControlCommand
from sim_compare.plotting import render_svg_from_csv
from sim_compare.visualization import CarlaCameraDisplay


class ExperimentRunner:
    def __init__(self, config: ExperimentConfig):
        self.config = config

    def _poll_render_window_close(self, pygame_module) -> bool:
        for event in pygame_module.event.get():
            if event.type == pygame_module.QUIT:
                return True
            if (
                event.type == pygame_module.KEYUP
                and event.key == pygame_module.K_ESCAPE
            ):
                return True
        return False

    def run(self) -> int:
        cfg = self.config
        if cfg.input_mode == "series" and cfg.control_csv is None:
            print("--control-csv is required when --input-mode series", file=sys.stderr)
            return 2
        if cfg.dt <= 0.0:
            print("--dt must be > 0", file=sys.stderr)
            return 2
        if cfg.max_steps <= 0:
            print("--max-steps must be > 0", file=sys.stderr)
            return 2

        try:
            map_paths = resolve_map_paths(cfg.project_root, cfg.map_name)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 2

        esmini_lib = cfg.esmini_home / "bin" / "libesminiLib.so"
        if not esmini_lib.exists():
            print(f"esmini library not found: {esmini_lib}", file=sys.stderr)
            return 2

        render_carla = bool(cfg.render_carla or cfg.input_mode == "keyboard")
        search_paths = build_esmini_search_paths(
            cfg.esmini_home,
            map_paths,
            cfg.extra_esmini_paths,
        )

        pygame_module = None
        clock = None
        display = None
        control_source = None
        esmini = None
        carla = None
        csv_logger = None

        try:
            if render_carla:
                import pygame  # pylint: disable=import-outside-toplevel

                pygame.init()
                pygame.font.init()
                pygame.display.set_caption("CARLA/esmini compare")
                pygame_module = pygame
                clock = pygame.time.Clock()

            if cfg.input_mode == "series":
                control_source = SeriesControlSource(
                    cfg.control_csv,
                    cfg.hold_last_control,
                )
            else:
                if pygame_module is None:
                    raise RuntimeError("keyboard mode requires pygame initialization")
                control_source = KeyboardControlSource(pygame_module)

            max_steering_angle_rad = read_front_axle_max_steering_rad(
                map_paths.xosc_path,
                ego_name="Ego",
            )

            esmini = create_esmini_bridge(
                esmini_home=cfg.esmini_home,
                xosc_path=map_paths.xosc_path,
                search_paths=search_paths,
                ego_index=cfg.ego_index,
                options=cfg.esmini_options,
                backend=cfg.esmini_backend,
            )
            esmini.set_initial_pose(cfg.initial_pose)
            esmini.start()

            carla = CarlaBridge(
                host=cfg.carla_host,
                port=cfg.carla_port,
                timeout_s=cfg.carla_timeout,
                traffic_manager_port=cfg.traffic_manager_port,
                vehicle_filter=cfg.carla_vehicle_filter,
                xodr_path=map_paths.xodr_path,
                dt_s=cfg.dt,
                coordinate_transform=cfg.coordinate_transform,
                opendrive_config=cfg.carla_opendrive,
            )
            carla.spawn_vehicle(cfg.initial_pose)

            if render_carla:
                width, height = cfg.resolution
                display = CarlaCameraDisplay(
                    pygame_module,
                    carla.actor,
                    width,
                    height,
                    cfg.gamma,
                )

            csv_logger = ComparisonCsvLogger(cfg.csv_out)

            initial_carla_control = CarlaControlCommand()
            initial_esmini_control = carla_control_to_esmini_control(
                initial_carla_control,
                cfg.esmini_backend.mode,
                max_steering_angle_rad,
            )
            initial_carla_state = carla.get_state(0.0)
            initial_esmini_state = esmini.get_state(0.0)
            initial_diffs = csv_logger.write(
                0,
                initial_carla_control,
                initial_esmini_control,
                initial_carla_state,
                initial_esmini_state,
            )

            sum_abs_pos = initial_diffs["diff_pos_2d"]
            sum_abs_speed = abs(initial_diffs["diff_speed"])
            sum_abs_accel = abs(initial_diffs["diff_acceleration"])
            max_pos = initial_diffs["diff_pos_2d"]
            steps_done = 1
            sim_time_s = 0.0
            for step in range(1, cfg.max_steps + 1):
                milliseconds = clock.tick_busy_loop(60) if clock is not None else 0

                if display is not None and cfg.input_mode != "keyboard":
                    if self._poll_render_window_close(pygame_module):
                        break

                current_control, should_stop = control_source.next(
                    sim_time_s,
                    step - 1,
                    milliseconds,
                )
                if should_stop:
                    break

                esmini_control = carla_control_to_esmini_control(
                    current_control,
                    cfg.esmini_backend.mode,
                    max_steering_angle_rad,
                )
                next_sim_time_s = sim_time_s + cfg.dt
                carla_state = carla.step(current_control, next_sim_time_s)
                esmini_state = esmini.step(
                    esmini_control,
                    cfg.dt,
                    next_sim_time_s,
                )
                sim_time_s = next_sim_time_s

                diffs = csv_logger.write(
                    step,
                    current_control,
                    esmini_control,
                    carla_state,
                    esmini_state,
                )
                sum_abs_pos += diffs["diff_pos_2d"]
                sum_abs_speed += abs(diffs["diff_speed"])
                sum_abs_accel += abs(diffs["diff_acceleration"])
                max_pos = max(max_pos, diffs["diff_pos_2d"])
                steps_done += 1

                if cfg.print_every > 0 and step % cfg.print_every == 0:
                    print(
                        f"[{step:05d}] t={sim_time_s:6.2f}s "
                        f"esmini={cfg.esmini_backend.mode} "
                        f"ctrl(thr={current_control.throttle:.2f}, "
                        f"brk={current_control.brake:.2f}, "
                        f"str={current_control.steer:.2f}, "
                        f"rev={int(current_control.reverse)}) "
                        f"diff(pos2d={diffs['diff_pos_2d']:.3f}m, "
                        f"speed={diffs['diff_speed']:.3f}m/s, "
                        f"accel={diffs['diff_acceleration']:.3f}m/s^2)"
                    )

                if display is not None:
                    display.render(
                        [
                            f"map: {cfg.map_name}",
                            f"time: {sim_time_s:.2f}s",
                            f"esmini backend: {cfg.esmini_backend.mode}",
                            (
                                "ctrl: "
                                f"thr={current_control.throttle:.2f} "
                                f"brk={current_control.brake:.2f} "
                                f"str={current_control.steer:.2f} "
                                f"rev={int(current_control.reverse)} "
                                f"hb={int(current_control.hand_brake)}"
                            ),
                            (
                                "diff: "
                                f"pos2d={diffs['diff_pos_2d']:.3f}m "
                                f"speed={diffs['diff_speed']:.3f}m/s "
                                f"acc={diffs['diff_acceleration']:.3f}m/s^2"
                            ),
                        ]
                    )

                if esmini.should_quit():
                    break

            if steps_done == 0:
                print("No simulation step executed.")
            else:
                if csv_logger is not None:
                    csv_logger.close()
                    csv_logger = None
                plot_path = None
                if cfg.plot_out is not None:
                    plot_path = render_svg_from_csv(
                        cfg.csv_out,
                        output_path=cfg.plot_out,
                        title=f"CARLA vs esmini Trajectory: {cfg.map_name}",
                    )
                print(f"Done. map={cfg.map_name} log={cfg.csv_out}")
                if plot_path is not None:
                    print(f"Plot: {plot_path}")
                print(
                    "Summary: "
                    f"steps={steps_done}, "
                    f"mean|pos2d|={sum_abs_pos / steps_done:.4f} m, "
                    f"max|pos2d|={max_pos:.4f} m, "
                    f"mean|speed|={sum_abs_speed / steps_done:.4f} m/s, "
                    f"mean|accel|={sum_abs_accel / steps_done:.4f} m/s^2"
                )
            return 0
        finally:
            if csv_logger is not None:
                csv_logger.close()
            if display is not None:
                try:
                    display.destroy()
                except Exception:
                    pass
            if control_source is not None:
                try:
                    control_source.close()
                except Exception:
                    pass
            if carla is not None:
                try:
                    carla.close()
                except Exception:
                    pass
            if esmini is not None:
                try:
                    esmini.close()
                except Exception:
                    pass
            if pygame_module is not None:
                pygame_module.quit()
