from __future__ import annotations

import sys

from sim_compare.bridges.esmini_scenario import read_front_axle_max_steering_rad
from sim_compare.config import ExperimentConfig
from sim_compare.control_mapping import (
    ControlMappingContext,
    get_control_mapper,
    normalized_to_control_space,
)
from sim_compare.control_spaces import normalize_control_space
from sim_compare.controls.sources import KeyboardControlSource, SeriesControlSource
from sim_compare.logging import ComparisonCsvLogger
from sim_compare.maps import build_esmini_search_paths, resolve_map_paths
from sim_compare.models import InputControlCommand
from sim_compare.plotting import render_svg_from_csv
from sim_compare.simulators import build_simulator_adapter, make_csv_prefix
from sim_compare.simulators.base import SimulatorAdapter
from sim_compare.visualization import CarlaCameraDisplay


class ExperimentRunner:
    def __init__(self, config: ExperimentConfig):
        self.config = config

    def _build_simulators(
        self,
        map_paths,
        search_paths,
    ) -> tuple[SimulatorAdapter, SimulatorAdapter]:
        cfg = self.config
        reference = build_simulator_adapter(
            cfg.reference_simulator.simulator_id,
            cfg,
            map_paths,
            search_paths,
            label=cfg.reference_simulator.label,
        )
        candidate = build_simulator_adapter(
            cfg.candidate_simulator.simulator_id,
            cfg,
            map_paths,
            search_paths,
            label=cfg.candidate_simulator.label,
        )
        return reference, candidate

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
        control_source_space = normalize_control_space(cfg.control_source_space)
        if cfg.input_mode == "series" and cfg.control_csv is None:
            print("--control-csv is required when --input-mode series", file=sys.stderr)
            return 2
        if cfg.dt <= 0.0:
            print("--dt must be > 0", file=sys.stderr)
            return 2
        if cfg.max_steps <= 0:
            print("--max-steps must be > 0", file=sys.stderr)
            return 2
        if cfg.reference_simulator.simulator_id == cfg.candidate_simulator.simulator_id:
            print(
                "Comparing two instances of the same simulator is not supported by the current CLI/config yet.",
                file=sys.stderr,
            )
            return 2
        if make_csv_prefix(cfg.reference_simulator.label) == make_csv_prefix(
            cfg.candidate_simulator.label
        ):
            print(
                "Reference and candidate labels resolve to the same CSV prefix. "
                "Choose distinct --reference-label / --candidate-label values.",
                file=sys.stderr,
            )
            return 2

        try:
            map_paths = resolve_map_paths(cfg.project_root, cfg.map_name)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 2

        requires_esmini = "esmini" in {
            cfg.reference_simulator.simulator_id,
            cfg.candidate_simulator.simulator_id,
        }
        if requires_esmini:
            esmini_lib = cfg.esmini_home / "bin" / "libesminiLib.so"
            if not esmini_lib.exists():
                print(f"esmini library not found: {esmini_lib}", file=sys.stderr)
                return 2

        render_camera = bool(cfg.render_camera or cfg.input_mode == "keyboard")
        search_paths = (
            build_esmini_search_paths(
                cfg.esmini_home,
                map_paths,
                cfg.extra_esmini_paths,
            )
            if requires_esmini
            else []
        )

        pygame_module = None
        clock = None
        display = None
        control_source = None
        simulators: list[SimulatorAdapter] = []
        csv_logger = None

        try:
            if render_camera:
                import pygame  # pylint: disable=import-outside-toplevel

                pygame.init()
                pygame.font.init()
                pygame.display.set_caption("Simulator compare")
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
            mapping_context = ControlMappingContext(
                max_steering_angle_rad=max_steering_angle_rad
            )
            control_mapper = get_control_mapper(cfg.control_mapping_strategy)

            reference, candidate = self._build_simulators(map_paths, search_paths)
            simulators = [reference, candidate]
            for simulator in simulators:
                simulator.start(cfg.initial_pose)

            if render_camera:
                render_target = next(
                    (
                        simulator.get_render_actor()
                        for simulator in simulators
                        if simulator.get_render_actor() is not None
                    ),
                    None,
                )
                if render_target is not None:
                    width, height = cfg.resolution
                    display = CarlaCameraDisplay(
                        pygame_module,
                        render_target,
                        width,
                        height,
                        cfg.gamma,
                    )

            csv_logger = ComparisonCsvLogger(
                cfg.csv_out,
                reference=reference.descriptor,
                candidate=candidate.descriptor,
            )

            initial_input_control = InputControlCommand()
            source_native_control = normalized_to_control_space(
                control_source_space,
                initial_input_control,
                mapping_context,
            )
            initial_reference_control = control_mapper.map(
                source_native_control,
                reference.descriptor.control_space,
                mapping_context,
            )
            initial_candidate_control = control_mapper.map(
                source_native_control,
                candidate.descriptor.control_space,
                mapping_context,
            )
            initial_reference_state = reference.get_state(0.0)
            initial_candidate_state = candidate.get_state(0.0)
            initial_diffs = csv_logger.write(
                0,
                initial_input_control,
                control_source_space,
                control_mapper.name,
                initial_reference_control,
                initial_candidate_control,
                initial_reference_state,
                initial_candidate_state,
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

                source_native_control = normalized_to_control_space(
                    control_source_space,
                    current_control,
                    mapping_context,
                )
                reference_control = control_mapper.map(
                    source_native_control,
                    reference.descriptor.control_space,
                    mapping_context,
                )
                candidate_control = control_mapper.map(
                    source_native_control,
                    candidate.descriptor.control_space,
                    mapping_context,
                )
                next_sim_time_s = sim_time_s + cfg.dt
                reference_state = reference.step(
                    reference_control,
                    cfg.dt,
                    next_sim_time_s,
                )
                candidate_state = candidate.step(
                    candidate_control,
                    cfg.dt,
                    next_sim_time_s,
                )
                sim_time_s = next_sim_time_s

                diffs = csv_logger.write(
                    step,
                    current_control,
                    control_source_space,
                    control_mapper.name,
                    reference_control,
                    candidate_control,
                    reference_state,
                    candidate_state,
                )
                sum_abs_pos += diffs["diff_pos_2d"]
                sum_abs_speed += abs(diffs["diff_speed"])
                sum_abs_accel += abs(diffs["diff_acceleration"])
                max_pos = max(max_pos, diffs["diff_pos_2d"])
                steps_done += 1

                if cfg.print_every > 0 and step % cfg.print_every == 0:
                    print(
                        f"[{step:05d}] t={sim_time_s:6.2f}s "
                        f"ref={reference.descriptor.label}({reference.descriptor.backend_name}) "
                        f"cand={candidate.descriptor.label}({candidate.descriptor.backend_name}) "
                        f"src={control_source_space} "
                        f"map={control_mapper.name} "
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
                            (
                                f"ref: {reference.descriptor.label} "
                                f"({reference.descriptor.simulator_id}, {reference.descriptor.backend_name})"
                            ),
                            (
                                f"cand: {candidate.descriptor.label} "
                                f"({candidate.descriptor.simulator_id}, {candidate.descriptor.backend_name})"
                            ),
                            f"source space: {control_source_space}",
                            f"mapping: {control_mapper.name}",
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

                if any(simulator.should_quit() for simulator in simulators):
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
                        title=(
                            f"{reference.descriptor.label} vs "
                            f"{candidate.descriptor.label} Trajectory: {cfg.map_name}"
                        ),
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
            for simulator in reversed(simulators):
                try:
                    simulator.close()
                except Exception:
                    pass
            if pygame_module is not None:
                pygame_module.quit()
