# Control Mapping Framework

## Goal

The comparison framework now separates:

- external input control
- source control interpretation
- cross-simulator control mapping
- simulator-native control application

This is intended to support:

- `CARLA -> esmini`
- `esmini -> CARLA`
- future simulator additions without rewriting the main experiment loop

## Main Concepts

### 1. Input control

File:

- `sim_compare/models.py`

Type:

- `InputControlCommand`

This is the external control coming from:

- keyboard
- control CSV

It is a normalized actuation command:

- `throttle`
- `brake`
- `steer`
- `hand_brake`
- `reverse`

### 2. Control space

File:

- `sim_compare/control_spaces.py`

Current control spaces:

- `canonical.actuation`
- `carla.vehicle`
- `esmini.simple_vehicle`
- `esmini.bcs_controller`

These describe how a control vector should be interpreted.

### 3. Applied control

File:

- `sim_compare/models.py`

Type:

- `AppliedControlCommand`

This is the native control actually sent to a simulator/backend.

It includes a superset of fields:

- `throttle`
- `brake`
- `steer`
- `pedal`
- `steering_angle_rad`
- `hand_brake`
- `reverse`
- `control_space`

### 4. Mapping strategy

File:

- `sim_compare/control_mapping.py`

Current strategies:

- `semantic_roundtrip`
- `direct_numeric`

#### `semantic_roundtrip`

Pipeline:

1. interpret source-native control in semantic normalized space
2. rebuild target-native control from that normalized control

This is the default and is the cleanest baseline for physical comparison.

#### `direct_numeric`

Pipeline:

1. take the source-native numbers more directly
2. map them into the target-native fields with minimal semantic correction

This is useful for studying cases like:

- "what happens if the control intended for esmini is sent directly to CARLA?"

## Experiment Pipeline

The runner now uses this sequence:

1. load `InputControlCommand`
2. interpret it in `control_source_space`
3. build `source_native_control`
4. map to:
   - CARLA native control
   - esmini native control
5. step both simulators
6. log:
   - input control
   - source control space
   - mapping strategy
   - per-simulator applied control
   - per-simulator state
   - state differences

## New CLI Arguments

- `--control-source-space`
- `--control-mapping-strategy`

Examples:

### Default semantic comparison

```bash
--control-source-space canonical.actuation \
--control-mapping-strategy semantic_roundtrip
```

### Approximate current CARLA-first behavior

```bash
--control-source-space carla.vehicle \
--control-mapping-strategy semantic_roundtrip
```

### Study "esmini simple control sent directly to CARLA"

```bash
--control-source-space esmini.simple_vehicle \
--control-mapping-strategy direct_numeric
```

### Study "BCS control sent directly to CARLA"

```bash
--control-source-space esmini.bcs_controller \
--control-mapping-strategy direct_numeric
```

## Extending to Another Simulator

To add a new simulator cleanly:

1. define a new control space in `sim_compare/control_spaces.py`
2. add conversion rules in `sim_compare/control_mapping.py`
3. implement the low-level bridge under `sim_compare/bridges/` if needed
4. implement a simulator adapter under `sim_compare/simulators/`
5. register it in `sim_compare/simulators/factory.py`
6. wire any simulator-specific config / CLI options into `ExperimentConfig`

The runner now works with a reference/candidate pair of simulator adapters, so adding a simulator no longer requires editing the core stepping loop unless the experiment model itself changes.
