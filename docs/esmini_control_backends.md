# esmini Control Backend Notes

## Goal

The original research goal was:

- run CARLA and esmini in the same environment
- apply the same control to both
- compare resulting vehicle states
- allow esmini to respect the vehicle parameters defined in OpenSCENARIO as much as possible

This document records what was found and how that affects the current implementation.

## Current Implemented esmini Backends

### 1. `simple_vehicle_api`

Implementation:

- `sim_compare/bridges/esmini_simple_vehicle.py`

Mechanism:

- create `SE_SimpleVehicle`
- drive it through `SE_SimpleVehicleControlAnalog`
- report the resulting state back to the scenario object each timestep
- optionally override simple vehicle parameters from an external TOML profile

Advantages:

- straightforward external control
- easy mapping from CARLA throttle/brake/steer
- no scenario rewriting required
- tunable without editing the `.xosc`

Disadvantages:

- vehicle behavior comes from esmini simple vehicle model
- does not faithfully follow OpenSCENARIO entity `Performance` parameters
- public API exposure is limited compared with the BCS controller path

Use this backend when:

- you want the old behavior
- you need a simple baseline
- you want to calibrate esmini vehicle response from external config files

### Parameters exposed by the simple vehicle API

The current framework can override these simple-vehicle attributes:

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

Important limitation:

- the public simple vehicle API does not expose axle geometry, max steering angle, wheel radius, or center-of-gravity placement

### 2. `bcs_controller`

Implementation:

- `sim_compare/bridges/esmini_bcs_controller.py`
- `sim_compare/bridges/esmini_scenario.py`

Mechanism:

- generate runtime `.xosc`
- assign Ego `ObjectController` to `BCSController`
- send UDP `driverInput` packets every step
- let the esmini controller configure its internal vehicle model from the OpenSCENARIO Ego vehicle

Advantages:

- goes through an OpenSCENARIO controller assignment path
- matches the framework goal of backend-swappable esmini control more closely
- does not require direct `SE_SimpleVehicle` ownership in the test harness
- uses scenario-defined longitudinal limits and steering/geometry inputs

Disadvantages:

- reverse is not represented equivalently in the current path
- still uses esmini's simplified bicycle-model driver-input path rather than a full high-fidelity vehicle model

Use this backend when:

- you want to test a controller-driven esmini integration
- you want to keep esmini control inside the scenario/controller path

## Why `InteractiveController` Was Not Used as the Runtime Backend

`InteractiveController` is important because it is the closest built-in controller to your original requirement.

What the source shows:

- it sets max speed from `object_->GetMaxSpeed()`
- it sets max acceleration from `object_->GetMaxAcceleration()`
- it sets max deceleration from `object_->GetMaxDeceleration()`

This means it is aware of the entity parameters defined in the OpenSCENARIO vehicle.

However, for this framework we need a way to inject external control from Python each timestep.

The blocking issue is:

- no suitable public esmini C API was identified for injecting the same controller key events from the outside

So even though `InteractiveController` is conceptually attractive, it is not currently usable as the synchronized external-control backend in this Python framework.

## Why `ExternalController` Was Not Used

`ExternalController` is not a throttle/brake/steer controller.

Its role is closer to:

- external state reporting
- ghost support
- external simulator integration through object state updates

That does not solve the actual research problem of:

- issuing equivalent control commands to CARLA and esmini
- letting esmini internal vehicle/controller dynamics respond to those commands

## What `BCSController` Changed in the esmini Fork

The forked esmini now adds a new embedded controller:

- `BCSController`

It starts from the `UDPDriverController` driver-input path, but removes the hardcoded vehicle limits and instead configures the internal driver-input vehicle from the Ego `ScenarioObject`.

### OpenSCENARIO parameters now used by `BCSController`

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

These are mapped into:

- max longitudinal acceleration
- max longitudinal deceleration
- max speed
- vehicle length
- max wheel angle
- wheelbase
- rear-axle-to-CG distance
- wheel radius for wheel rotation

### Parameters not directly used in the current bicycle dynamics

- `Vehicle/@mass`
- axle `@trackWidth`
- axle `@positionZ`

These values still exist in the scenario object, but the current bicycle-model implementation does not consume them in its planar dynamics update.

## Why `BCSController` Is Still Only a Partial Solution

`BCSController` now fixes the most immediate problem in the stock `UDPDriverController` path:

- it no longer hardcodes max speed / max acceleration
- it reads relevant vehicle parameters from OpenSCENARIO

But it is still not the same as a full vehicle dynamics model.

The remaining limitation is structural:

- it still drives esmini's internal bicycle-model style driver-input vehicle

That means:

- it is much closer to the scenario-defined vehicle than the old stock UDP driver path
- but it is still not equivalent to a richer tire / suspension / drivetrain model

## `OverrideControllerValueAction`

Conceptually, this is the most promising direction.

Why:

- it is explicitly part of OpenSCENARIO controller action semantics
- it is the right abstraction for overriding throttle, brake and steering wheel values while keeping the controller/vehicle path inside the scenario engine

What was found:

- the public esmini API exposes `SE_GetOverrideActionStatus(...)`
- no matching public setter was found during inspection

So the current limitation is:

- the framework can observe override action status
- but it cannot dynamically write override controller values at each timestep through the public library API

## Practical Conclusion

With the current fork, the framework now has a usable controller-based path that supports:

1. external timestep-by-timestep control from Python
2. controller-driven motion inside esmini
3. use of key OpenSCENARIO Ego vehicle parameters

The framework currently exposes two practical choices:

- `simple_vehicle_api`
- `bcs_controller`

The tradeoff is now:

- `simple_vehicle_api`: easiest direct API baseline, now tunable through external vehicle profiles
- `bcs_controller`: controller-driven baseline that respects scenario-defined limits and geometry better

It still leaves space for future backends if you later need a different control interface.

## Additional Future Directions

If you later need even tighter controller semantics, the two most promising follow-up directions are:

### Option 1. Public key-event injection for `InteractiveController`

Expose a C API like:

```c
SE_ReportKeyEvent(int object_id, int key, bool down)
```

Then:

- keep `InteractiveController`
- map CARLA control to key/button style commands
- let the controller continue using entity parameters internally

### Option 2. Public runtime setter for override controller values

Expose a C API like:

```c
SE_SetOverrideActionStatus(int object_id, const SE_OverrideActionList *list)
```

Then:

- keep controller semantics in the scenario engine
- override throttle/brake/steering wheel each timestep from Python
- stay much closer to the intended OpenSCENARIO abstraction

## What the Current Framework Already Makes Easier

Even though the final esmini path is not complete yet, the refactor already helps:

- esmini backend selection is explicit
- control mapping is decoupled from simulator stepping
- scenario rewriting is isolated in one module
- CSV logging can record backend-specific control fields
- future esmini patches can be added as a new backend with limited changes to the rest of the codebase

## How to Switch Backends

### Old path

```bash
--esmini-backend simple_vehicle_api
```

### Controller path

```bash
--esmini-backend bcs_controller
```

The CLI still accepts:

```bash
--esmini-backend udp_driver_controller
```

as a backward-compatible alias.

## Suggested Interpretation for Experiments

When analyzing results, do not treat the two esmini backends as interchangeable physically.

Interpretation should be:

- `simple_vehicle_api`: API-driven baseline
- `bcs_controller`: controller-path baseline

They are both useful, but they answer different questions.
