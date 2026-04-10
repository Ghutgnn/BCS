# Simulator Comparison Framework

## Purpose

This framework runs a pair of simulator adapters on the same map, with the same initial pose and a synchronized fixed time step, then records how their vehicle states diverge under the same input stream.

The current built-in adapters are:

- `carla`
- `esmini`

The current default experiment remains `carla` vs `esmini`, but the comparison loop is no longer hardcoded to those two implementations.

## Core Design

### 1. Pairwise comparison, not simulator-specific runner logic

The runner now operates on two simulator adapters:

- `reference`
- `candidate`

The `reference` simulator defines:

- the sign of `diff_*` values in CSV
- the first trajectory in the generated SVG
- the first simulator metadata block in the CSV

The `candidate` simulator is compared against that reference.

The runner itself no longer constructs `CarlaBridge` and `esmini` bridges directly. It asks the simulator factory to build adapters from config.

### 2. Layered architecture

The code is organized into three layers.

#### Control source layer

`sim_compare/controls/sources.py`

This layer answers:

- where control input comes from
- how each timestep gets the next input sample

Current sources:

- `SeriesControlSource`
- `KeyboardControlSource`

#### Control interpretation and mapping layer

- `sim_compare/control_spaces.py`
- `sim_compare/control_mapping.py`
- `docs/control_mapping_framework.md`

This layer answers:

- what control space the external input is expressed in
- how that control should be mapped into each simulator's native control space

Key dataclasses:

- `InputControlCommand`
- `AppliedControlCommand`

#### Simulator adapter layer

- `sim_compare/simulators/base.py`
- `sim_compare/simulators/factory.py`
- `sim_compare/simulators/carla.py`
- `sim_compare/simulators/esmini.py`

This layer answers:

- how a simulator instance is built
- how it is started/reset to the initial pose
- how one timestep is executed
- how to fetch state
- what native control space it expects

Under the adapter layer, low-level simulator-specific bridge code still lives in `sim_compare/bridges/`.

## Project Layout

```text
BCS/
├── carla_esmini_compare.py
├── plot_comparison.py
├── configs/
├── docs/
├── maps/
└── sim_compare/
    ├── bridges/
    ├── controls/
    ├── simulators/
    ├── config.py
    ├── control_mapping.py
    ├── control_spaces.py
    ├── esmini_backend.py
    ├── logging.py
    ├── maps.py
    ├── models.py
    ├── plotting.py
    ├── runner.py
    ├── simple_vehicle_config.py
    ├── utils.py
    └── visualization.py
```

## Main Modules

### Entry points

- `carla_esmini_compare.py`
  - parses CLI arguments
  - builds `ExperimentConfig`
  - runs `ExperimentRunner`
- `plot_comparison.py`
  - regenerates trajectory SVG from an existing CSV

### Configuration

`sim_compare/config.py`

Important config groups:

- `ExperimentConfig`
- `SimulatorInstanceConfig`
- `EsminiBackendConfig`
- `EsminiOptions`
- `CarlaOpenDriveGenerationConfig`
- `CoordinateTransformConfig`

### Maps

`sim_compare/maps.py`

The framework still resolves maps by shared `map_name`:

- esmini scenario: `maps/[map_name].xosc`
- CARLA OpenDRIVE map: `maps/[map_name].xodr`

### Simulator factory and adapters

- `sim_compare/simulators/factory.py`
- `sim_compare/simulators/base.py`

The factory exposes:

- `SUPPORTED_SIMULATORS`
- `normalize_simulator_id()`
- `build_simulator_adapter()`

The adapter contract is defined by `SimulatorAdapter` in `sim_compare/simulators/base.py`.

Each adapter owns a `SimulatorDescriptor`, which records:

- `simulator_id`
- `label`
- `csv_prefix`
- `control_space`
- `backend_name`

### Low-level bridges

#### CARLA

- `sim_compare/bridges/carla.py`

Responsibilities:

- load `.xodr`
- generate OpenDRIVE world
- enable synchronous stepping
- spawn and reset ego vehicle
- apply CARLA `VehicleControl`
- read vehicle state

#### esmini runtime/common

- `sim_compare/bridges/esmini_runtime.py`

Responsibilities:

- load `libesminiLib.so`
- configure esmini options
- initialize scenario
- report pose/state to esmini APIs
- read ego state from esmini

#### esmini simple vehicle backend

- `sim_compare/bridges/esmini_simple_vehicle.py`

This backend:

- creates `SE_SimpleVehicle`
- can override basic simple-vehicle parameters from a TOML profile
- drives the simple vehicle through esmini C API calls each timestep
- reports the simple vehicle pose back to the scenario ego object

#### esmini controller-driven backend

- `sim_compare/bridges/esmini_bcs_controller.py`
- `sim_compare/bridges/esmini_scenario.py`

This backend:

- generates a runtime `.xosc`
- injects the `BCSController`
- activates the controller in the scenario `Init`
- sends driver input over UDP each timestep

The old module name `sim_compare/bridges/esmini_udp_driver.py` remains as a compatibility shim.

## Control Model

### Input control

`InputControlCommand` represents the external experiment input before simulator-specific mapping.

Fields:

- `throttle`
- `brake`
- `steer`
- `hand_brake`
- `reverse`

### Applied control

`AppliedControlCommand` represents simulator-native control after mapping.

It carries generic fields plus backend-specific ones such as:

- `pedal`
- `steering_angle_rad`
- `control_space`

## Logging

`sim_compare/logging.py`

The CSV schema is now generated from simulator descriptors instead of being hardcoded to `carla_*` and `esmini_*` field groups.

The CSV always contains:

- input-control columns
- control-source metadata
- control-mapping metadata
- reference/candidate simulator metadata
- per-simulator control columns
- per-simulator state columns
- `diff_*` columns

Important metadata columns:

- `reference_simulator_id`
- `reference_label`
- `reference_csv_prefix`
- `reference_backend`
- `candidate_simulator_id`
- `candidate_label`
- `candidate_csv_prefix`
- `candidate_backend`

For the default experiment, the prefixes are still usually:

- `carla`
- `esmini`

but the logger no longer depends on those exact names.

## Plotting

`sim_compare/plotting.py`

The SVG plotter now reads simulator metadata from the CSV and uses the logged prefixes/labels instead of assuming fixed `carla` / `esmini` column names.

It shows:

- both trajectories projected onto one XY plane
- color intensity as speed encoding
- start and end markers
- mean/max/final position difference summary

## CLI

Main command:

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py MAP_NAME [options]
```

Important options:

- `--reference-simulator {carla,esmini}`
- `--candidate-simulator {carla,esmini}`
- `--reference-label LABEL`
- `--candidate-label LABEL`
- `--input-mode {series,keyboard}`
- `--control-source-space ...`
- `--control-mapping-strategy ...`
- `--esmini-backend {simple_vehicle_api,bcs_controller,udp_driver_controller}`
- `--simple-vehicle-config PATH`
- `--render-camera`

Notes:

- comparing two instances of the same simulator is not supported yet by the current CLI/config
- `--render-carla` remains accepted as an alias of `--render-camera`
- `udp_driver_controller` remains accepted as a backward-compatible alias of `bcs_controller`

## Typical workflows

### Default CARLA vs esmini simple vehicle

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  ground \
  --init-x 0 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --input-mode keyboard \
  --esmini-backend simple_vehicle_api
```

### CARLA vs esmini simple vehicle with tuned profile

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  ground \
  --init-x 0 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --input-mode series \
  --control-csv controls.csv \
  --esmini-backend simple_vehicle_api \
  --simple-vehicle-config configs/vehicles/carla_tesla_model3.toml
```

### CARLA vs esmini BCS controller backend

```bash
python /home/hcis-s05/ysws/BCS/carla_esmini_compare.py \
  ground \
  --init-x 0 \
  --init-y 0 \
  --init-yaw-deg 0 \
  --input-mode keyboard \
  --esmini-backend bcs_controller
```

## Extending the framework

### Add a new simulator

1. Implement a low-level bridge under `sim_compare/bridges/` if needed.
2. Implement a simulator adapter under `sim_compare/simulators/`.
3. Register it in `SIMULATOR_BUILDERS` inside `sim_compare/simulators/factory.py`.
4. Add any simulator-specific config/CLI options.
5. Add control-space support in `sim_compare/control_spaces.py` and `sim_compare/control_mapping.py` if the new simulator needs a new native control representation.

### Add a new esmini backend

1. Implement the backend bridge in `sim_compare/bridges/`.
2. Register it in `create_esmini_bridge()`.
3. Map backend name to control space in `sim_compare/esmini_backend.py` if needed.
4. Document the backend in `docs/esmini_control_backends.md`.

## Files to read first

1. `carla_esmini_compare.py`
2. `sim_compare/runner.py`
3. `sim_compare/simulators/base.py`
4. `sim_compare/simulators/factory.py`
5. `sim_compare/control_mapping.py`
6. `docs/control_mapping_framework.md`
7. `docs/esmini_control_backends.md`
