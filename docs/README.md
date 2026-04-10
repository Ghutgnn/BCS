# CARLA / esmini Comparison Framework

## Purpose

This framework runs CARLA and esmini on the same map, with the same initial pose and the same control input, then logs and visualizes how much the two simulators diverge over time.

The current implementation focuses on:

- shared map naming under `maps/`
- synchronized fixed-step execution
- reusable control-source and control-bridge modules
- swappable esmini control backends
- per-step CSV logging
- automatic trajectory visualization as SVG

## Project Layout

```text
BCS/
├── carla_esmini_compare.py
├── plot_comparison.py
├── maps/
├── examples/
├── docs/
└── sim_compare/
    ├── bridges/
    ├── controls/
    ├── config.py
    ├── logging.py
    ├── maps.py
    ├── models.py
    ├── plotting.py
    ├── runner.py
    ├── utils.py
    └── visualization.py
```

## Core Design

### 1. Map Resolution

You only pass `map_name`.

The framework resolves:

- esmini scenario: `maps/[map_name].xosc`
- CARLA OpenDRIVE map: `maps/[map_name].xodr`

This is implemented in `sim_compare/maps.py`.

### 2. Fixed-Step Synchronization

Both simulators are stepped with the same `dt`.

- CARLA uses synchronous mode with `fixed_delta_seconds = dt`
- esmini uses `SE_StepDT(dt)`

The main loop is in `sim_compare/runner.py`.

### 3. Canonical Control Flow

The framework uses one canonical control representation first:

- `CarlaControlCommand`

Then it maps that control into backend-specific esmini control:

- `EsminiControlCommand`

This makes it possible to keep adding more simulators or more esmini backends later.

### 4. Backend-Oriented esmini Integration

esmini is not hardcoded to a single control path anymore.

Current esmini backends:

- `simple_vehicle_api`
- `bcs_controller`
- `udp_driver_controller` (backward-compatible alias of `bcs_controller`)

The backend selection is centralized in `sim_compare/bridges/esmini.py`.

## Main Modules

### CLI Entry Points

- `carla_esmini_compare.py`
  - main experiment entry point
  - parses CLI arguments
  - builds `ExperimentConfig`
  - runs `ExperimentRunner`

- `plot_comparison.py`
  - renders an SVG trajectory plot from an existing comparison CSV

### Configuration and Data Models

- `sim_compare/config.py`
  - experiment config
  - CARLA OpenDRIVE generation config
  - esmini runtime options
  - esmini backend config

- `sim_compare/models.py`
  - canonical control/state dataclasses

### Maps

- `sim_compare/maps.py`
  - resolves `[map_name].xosc` and `[map_name].xodr`
  - builds esmini search paths

### Control Sources

- `sim_compare/controls/sources.py`
  - `SeriesControlSource`
  - `KeyboardControlSource`

Current supported input modes:

- `series`
- `keyboard`

### Control Bridges

- `sim_compare/controls/bridge.py`

Current mapping functions:

- `carla_control_to_esmini_simple_vehicle()`
- `carla_control_to_esmini_bcs_controller()`
- `carla_control_to_esmini_control()`

This is the intended extension point if you want to test another control mapping strategy.

### Simulator Bridges

#### CARLA

- `sim_compare/bridges/carla.py`

Responsibilities:

- load `.xodr` through `generate_opendrive_world()`
- set synchronous mode
- spawn ego vehicle
- apply CARLA control each tick
- return ego vehicle state

#### esmini runtime/common

- `sim_compare/bridges/esmini_runtime.py`

Responsibilities:

- load `libesminiLib.so`
- set esmini options
- initialize scenario
- fetch ego object state
- expose common low-level APIs to specific esmini backends

#### esmini simple vehicle backend

- `sim_compare/bridges/esmini_simple_vehicle.py`

Behavior:

- disables esmini controllers
- creates a `SE_SimpleVehicle`
- each step:
  - updates the simple vehicle with analog input
  - reports pose/speed/wheel state back to the esmini entity

This preserves the original API-driven path you started with.

#### esmini BCS controller backend

- `sim_compare/bridges/esmini_udp_driver.py`
- `sim_compare/bridges/esmini_scenario.py`

Behavior:

- generates a runtime `.xosc`
- replaces/creates Ego `ObjectController` with `BCSController`
- writes initial pose and initial speed into `Init`
- activates longitudinal and lateral controller domains
- sends `driverInput` packets over UDP every step
- lets esmini read vehicle limits and geometry from the OpenSCENARIO Ego vehicle definition

This is the current controller-based path exposed by the framework.

### Logging

- `sim_compare/logging.py`

Responsibilities:

- write one CSV row per step
- log:
  - CARLA control
  - backend-specific esmini control
  - CARLA ego state
  - esmini ego state
  - state differences

### Visualization

- `sim_compare/plotting.py`
  - renders trajectory SVG
- `sim_compare/visualization.py`
  - CARLA camera display via pygame

The SVG plot:

- projects CARLA and esmini trajectories onto the same XY plane
- uses different colors for the two simulators
- uses color intensity to encode speed
- marks start and end positions
- shows summary statistics for positional deviation

## Experiment Execution Flow

1. Parse CLI arguments in `carla_esmini_compare.py`
2. Build `ExperimentConfig`
3. Resolve map files from `maps/[map_name].xosc` and `maps/[map_name].xodr`
4. Initialize pygame if needed
5. Build control source:
   - `series`
   - `keyboard`
6. Read front axle max steering from the `.xosc`
7. Create esmini backend
8. Set esmini initial pose
9. Start esmini
10. Start CARLA and spawn the ego vehicle
11. For each step:
   - read control source
   - map CARLA control to esmini control
   - apply control to CARLA
   - apply control to esmini
   - log both states and their differences
   - update CARLA display if enabled
12. Close all resources
13. Render SVG from the generated CSV

## Available esmini Backends

### `simple_vehicle_api`

Use when:

- you want the original direct API path
- you need the simplest control loop
- you want a baseline for comparison

Characteristics:

- uses `SE_SimpleVehicleControlAnalog`
- control is mapped to:
  - `pedal`
  - `steer`
- ego entity is attached to the simple vehicle each step
- can optionally load a TOML vehicle profile and override simple-vehicle parameters via esmini API setters

Tradeoff:

- easy to control
- tunable from external config
- but still limited to what the simple vehicle API exposes

### `bcs_controller`

Use when:

- you want esmini control to go through an OpenSCENARIO controller path
- you want the test harness to exercise controller assignment and runtime scenario generation

Characteristics:

- creates runtime scenario file:
  - `maps/[map_name].runtime_bcs.xosc`
- uses `BCSController` with `driverInput`
- sends:
  - throttle
  - brake
  - steering wheel angle

Important limitation:

- reverse is still not represented equivalently to CARLA
- see `docs/esmini_control_backends.md`

## Current Input Modes

### `series`

Reads a CSV control sequence.

Minimum required columns:

- `throttle`
- `steer`

Optional columns:

- `time_s`
- `brake`
- `hand_brake`
- `reverse`

Behavior:

- if `time_s` exists, control is sampled by time
- otherwise control is sampled by row index

### `keyboard`

Manual keyboard control modeled after CARLA `manual_control.py`.

Current bindings:

- `W` / `Up`: throttle
- `S` / `Down`: brake
- `A` / `Left`: steer left
- `D` / `Right`: steer right
- `Q`: toggle reverse
- `Space`: hand brake
- `Esc`: quit

## CLI Reference

Main command:

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py MAP_NAME [options]
```

Important arguments:

- `map_name`
- `--init-x`
- `--init-y`
- `--init-yaw-deg`
- `--init-speed`
- `--carla-init-z`
- `--input-mode {series,keyboard}`
- `--control-csv`
- `--dt`
- `--max-steps`
- `--csv-out`
- `--plot-out`
- `--esmini-backend {simple_vehicle_api,bcs_controller,udp_driver_controller}`
- `--simple-vehicle-config`
- `--esmini-udp-base-port`
- `--esmini-udp-exec-mode`

Coordinate alignment arguments:

- `--invert-y`
- `--yaw-sign`
- `--yaw-offset-deg`

## Common Usage

### Run with the simple vehicle backend

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  straight_500m \
  --init-x 50 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --init-speed 0 \
  --input-mode keyboard \
  --esmini-backend simple_vehicle_api
```

### Run with the simple vehicle backend and an external vehicle profile

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  straight_500m \
  --init-x 50 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --init-speed 0 \
  --input-mode series \
  --control-csv /home/hcis-s05/ysws/BCS/control_series_example.csv \
  --esmini-backend simple_vehicle_api \
  --simple-vehicle-config configs/vehicles/carla_tesla_model3.toml
```

### Run with the BCS controller backend

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  straight_500m \
  --init-x 50 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --init-speed 0 \
  --input-mode series \
  --control-csv /home/hcis-s05/ysws/BCS/control_series_example.csv \
  --dt 0.05 \
  --esmini-backend bcs_controller \
  --esmini-udp-base-port 49950 \
  --esmini-udp-exec-mode synchronous
```

### Regenerate a plot from an existing CSV

```bash
python /home/hcis-s05/ysws/BCS/plot_comparison.py \
  /home/hcis-s05/ysws/BCS/comparison_straight_500m.csv
```

## Output Files

### CSV

Default:

- `comparison_[map_name].csv`

Contains:

- applied CARLA control
- backend-specific esmini control
- CARLA ego state
- esmini ego state
- state differences

Notable fields:

- `carla_control_*`
- `esmini_control_*`
- `esmini_control_backend`
- `carla_x`, `carla_y`, `carla_speed`, `carla_acceleration`
- `esmini_x`, `esmini_y`, `esmini_speed`, `esmini_acceleration`
- `diff_pos_2d`
- `diff_pos_3d`
- `diff_yaw`
- `diff_speed`
- `diff_acceleration`

### SVG

Default:

- `comparison_[map_name].svg`

Generated automatically after a run, and can also be generated later from CSV.

### esmini log

Default:

- `comparison_[map_name].esmini.log`

## Coordinate Alignment

The framework assumes both simulators can be compared in the same logical map frame, but in practice:

- Y axis direction may differ
- yaw sign may differ
- yaw zero reference may differ

Current correction parameters:

- `invert_y`
- `yaw_sign`
- `yaw_offset_deg`

These are applied in the CARLA bridge.

## Runtime-Generated Files

When using `bcs_controller`, the framework generates:

- `maps/[map_name].runtime_bcs.xosc`

This file is derived from the base `.xosc` and rewrites:

- Ego `ObjectController`
- Ego initial `TeleportAction`
- Ego initial `SpeedAction`
- Ego `ActivateControllerAction`

It is treated as a generated file and may be overwritten on the next run.

## Known Limitations

### esmini controller limitations

The most important technical limitation is:

- the framework can switch between different esmini control backends
- but the current public esmini API does not expose a clean runtime path for externally driving the built-in parameter-aware controller with arbitrary CARLA-equivalent control at each timestep

As a result:

- `simple_vehicle_api` is easy to control but not physically faithful
- `bcs_controller` uses scenario-defined limits and geometry, but it still inherits the simplified bicycle-model assumptions of esmini's driver-input path

Detailed notes are in `docs/esmini_control_backends.md`.

### Reverse gear mismatch

The current BCS controller backend warns and ignores reverse on the esmini side.

That means:

- CARLA reverse can be applied
- esmini reverse is not represented equivalently in this backend

### End-to-end validation

Static code validation has been done:

- Python syntax checks
- CSV-to-SVG plotting checks

But the framework still depends on your local simulator runtime environment for true end-to-end validation.

## Recommended Testing Workflow

### A. Baseline sanity test

1. Start CARLA server
2. Run with:
   - `--input-mode keyboard`
   - `--esmini-backend simple_vehicle_api`
3. Confirm:
   - CARLA window shows camera view
   - keyboard input is responsive
   - CSV and SVG are generated

### B. Controlled sequence test

1. Use a small `control_csv`
2. Run with:
   - `--input-mode series`
   - fixed `--dt`
   - fixed `--max-steps`
3. Compare:
   - trajectory divergence in SVG
   - numeric divergence in CSV

### C. Backend comparison test

1. Run the same experiment twice:
   - `simple_vehicle_api`
   - `bcs_controller`
2. Compare:
   - trajectory shape
   - speed profile
   - acceleration profile
   - sensitivity to steering and braking

### D. Coordinate calibration

If CARLA and esmini appear mirrored or rotated:

1. adjust `--invert-y`
2. adjust `--yaw-sign`
3. adjust `--yaw-offset-deg`
4. rerun until the initial direction and trajectory orientation align

## OpenSCENARIO Vehicle Parameters Used by `BCSController`

The esmini fork now maps these Ego vehicle fields into the internal driver-input bicycle model:

- `Vehicle/Performance/@maxSpeed`
- `Vehicle/Performance/@maxAcceleration`
- `Vehicle/Performance/@maxDeceleration`
- `Vehicle/BoundingBox/Dimensions/@length`
- `Vehicle/BoundingBox/Center/@x`
- `Vehicle/Axles/FrontAxle/@maxSteering`
- `Vehicle/Axles/FrontAxle/@positionX`
- `Vehicle/Axles/RearAxle/@positionX`
- `Vehicle/Axles/FrontAxle/@wheelDiameter`
- `Vehicle/Axles/RearAxle/@wheelDiameter`

These parameters now affect either:

- longitudinal limits
- maximum steering angle
- wheelbase / rear-axle-to-CG geometry
- wheel rotation radius

Vehicle parameters that still do not directly affect the current bicycle dynamics include:

- `@mass`
- axle `@trackWidth`
- axle `@positionZ`

## What the Simple Vehicle Profile Can Override

When using `--esmini-backend simple_vehicle_api`, you can now provide a TOML vehicle profile and override:

- `length_m`
- `max_speed_mps`
- `max_acceleration_mps2`
- `max_deceleration_mps2`
- `engine_brake_factor`
- `steering_scale`
- `steering_return_factor`
- `steering_rate`
- `throttle_disabled`
- `steering_disabled`

Reference example:

- `configs/vehicles/carla_tesla_model3.toml`

This path is useful when:

- you want external, simulator-side parameter control without editing the `.xosc`
- you want to calibrate a simple esmini vehicle against a specific CARLA vehicle

But note that the simple vehicle API still cannot directly override:

- axle geometry
- max steering angle
- wheel radius
- center-of-gravity placement

## Extension Points

### Add a new esmini backend

1. implement a new bridge under `sim_compare/bridges/`
2. keep the interface:
   - `set_initial_pose()`
   - `start()`
   - `step()`
   - `should_quit()`
   - `close()`
3. register it in `create_esmini_bridge()`
4. add a new control mapping in `sim_compare/controls/bridge.py`
5. add CLI selection in `carla_esmini_compare.py`

### Add a new simulator

The current code is structured so that a new simulator can be introduced by adding:

1. a new simulator bridge
2. a canonical-to-backend control mapping
3. integration in the runner

The data model and logging path are already separated enough to support that refactor.

## Files to Read First

If you are continuing development, start here:

1. `carla_esmini_compare.py`
2. `sim_compare/runner.py`
3. `sim_compare/controls/bridge.py`
4. `sim_compare/bridges/esmini.py`
5. `docs/esmini_control_backends.md`
