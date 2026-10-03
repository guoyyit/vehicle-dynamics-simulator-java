# Vehicle Dynamics Simulator

A deterministic vehicle dynamics simulation service for closed-loop validation of
upper-level control and planning algorithms. The service accepts vehicle parameters,
road conditions and time-varying driver inputs, and outputs the vehicle's motion state
(position, velocity, acceleration, heading angle and yaw rate).

## Features

- **Scenario-based simulation**: a scenario bundles vehicle parameters, road surface
  friction and a time-indexed driving schedule. Identical scenarios and configurations
  produce bit-identical state sequences.
- **Coupled longitudinal/lateral dynamics**: a planar single-track (bicycle) model with
  a linear tire model, friction ellipse force caps and quasi-static load transfer.
- **Adhesion limit enforcement**: per-axle friction ellipses cap the combined tire force
  to `mu * Fz`, guaranteeing the total acceleration never exceeds the tire-road limit.
- **Configurable step size and query times**: fixed-step RK4 integration with linear
  interpolation for arbitrary-time state queries.
- **Checkpoint/recovery**: states are persisted as JSON checkpoints at configurable
  intervals. Resuming from the latest checkpoint reproduces the one-shot trajectory
  point-by-point.
- **Concurrent execution**: each run operates on an independent simulator instance;
  identical scenarios run concurrently produce identical results.
- **Input validation and divergence detection**: illegal inputs, out-of-range parameters
  and numerical divergence throw explicit exceptions and terminate the scenario.

## Model Assumptions

### Coordinate System

- Body-fixed frame: x forward, y left, yaw counter-clockwise from x-axis.
- Position is in the ground plane (x, y).

### Vehicle Model

Planar single-track (bicycle) model with the following degrees of freedom:

- `vx`: longitudinal velocity (body frame)
- `vy`: lateral velocity (body frame)
- `r`: yaw rate (`yaw_dot`)
- `yaw`: heading angle
- `x, y`: ground-plane position

Equations of motion:

```
vx_dot = ax + r * vy
vy_dot = ay - r * vx
r_dot  = (lf * Fyf - lr * Fyr) / Iz
yaw_dot = r
x_dot  = vx * cos(yaw) - vy * sin(yaw)
y_dot  = vx * sin(yaw) + vy * cos(yaw)
```

### Tire Model

- Linear tire model: lateral force = cornering stiffness * slip angle.
  - Front: `Fyf = Cf * alpha_f`, where `alpha_f = delta - atan2(vy + lf*r, vx)`
  - Rear: `Fyr = Cr * alpha_r`, where `alpha_r = -atan2(vy - lr*r, vx)`
- Friction ellipse per axle: `sqrt(Fx^2 + Fy^2) <= mu * Fz`. If the combined force
  exceeds the limit, both components are scaled proportionally.
- Vertical loads include quasi-static longitudinal load transfer:
  - `Fzf = (m*g*lr - m*ax*h) / L`
  - `Fzr = (m*g*lf + m*ax*h) / L`

### Low-Speed Behavior

At low speeds the dynamic tire model is blended toward a kinematic bicycle model to
avoid slip-angle singularities:

- `beta = atan(lr * tan(delta) / L)`
- `vy_kin = vx * tan(beta)`
- `r_kin = vx * tan(delta) / L`

The blend weight is a smoothstep function of forward speed between 1 m/s (full
kinematic) and 4 m/s (full dynamic).

### Longential Force

- Rear-wheel drive: drive force goes to the rear axle.
- Braking force is split 60% front / 40% rear.
- Rolling resistance: `Frr = Crr * m * g`, opposing motion.
- Brake force is scaled by `tanh(vx / 0.5)` to prevent the vehicle from reversing
  when braking to a stop.

### Integration

- Fourth-order Runge-Kutta (RK4) with a fixed step size.
- Driver inputs are linearly interpolated from the schedule at each evaluation time.
- Step times are computed as `step * dt` to avoid floating-point accumulation.

## Parameters

### Vehicle Parameters (`VehicleParameters`)

| Parameter | Unit | Description |
|---|---|---|
| `mass` | kg | Vehicle mass |
| `wheelbase` | m | Wheelbase (front + rear axle distance) |
| `frontAxleDistance` | m | CG to front axle distance |
| `rearAxleDistance` | m | CG to rear axle distance |
| `cgHeight` | m | Center of gravity height |
| `yawInertia` | kg*m^2 | Yaw moment of inertia |
| `frontCorneringStiffness` | N/rad | Front tire cornering stiffness |
| `rearCorneringStiffness` | N/rad | Rear tire cornering stiffness |
| `maxSteerAngle` | rad | Maximum steering angle (at wheels) |
| `maxDriveForce` | N | Maximum drive force |
| `maxBrakeForce` | N | Maximum brake force |

### Road Condition (`RoadCondition`)

| Parameter | Unit | Description |
|---|---|---|
| `frictionCoefficient` | - | Tire-road friction coefficient (0 < mu <= 1) |
| `rollingResistanceCoefficient` | - | Rolling resistance coefficient (0 <= Crr < 1) |

### Driver Input (`DriverInput`)

| Parameter | Range | Description |
|---|---|---|
| `throttle` | [0, 1] | Throttle position |
| `brake` | [0, 1] | Brake position |
| `steeringAngle` | [-maxSteer, maxSteer] | Steering angle (at wheels) |

### Simulation Config (`SimulationConfig`)

| Parameter | Description |
|---|---|
| `stepSize` | Integration step size (dt) |
| `queryTimes` | Times at which to output interpolated states |
| `checkpointIntervalSteps` | Checkpoint interval (in steps) |

## Verification

### Automated Tests

Run the full test suite:

```bash
mvn test
```

Test coverage:

| Test Class | Coverage |
|---|---|
| `DeterminismTest` | Repeated runs are identical; concurrent repeated runs are identical; query states are deterministic |
| `FrictionLimitTest` | Acceleration never exceeds mu*g; low-mu produces lower accel than high-mu; no oscillation near limit; combined slip respects friction circle |
| `QueryConsistencyTest` | Query at step boundary returns exact step state; query at arbitrary time matches interpolation; query at 0/horizon returns boundary states; query outside horizon throws |
| `ResumeTest` | Resume matches one-shot point-by-point; resume from middle checkpoint; checkpoint timestamps aligned; multiple resume cycles identical |
| `ConcurrencyTest` | Multiple scenarios run concurrently without interference; same scenario concurrent runs identical; concurrent runs and resumes don't corrupt state |
| `ValidationTest` | Invalid parameters and inputs throw explicit exceptions |
| `BehaviorDifferenceTest` | Low-speed steering produces kinematic response; high-speed steering produces dynamic response; low-mu reduces cornering capability; vehicle stops without reversing |
| `SnapshotResumeTest` | Resume from persisted checkpoint matches one-shot; scenarios and checkpoints persist across restarts; multiple checkpoints ordered |
| `ScenarioControllerTest` | REST API endpoints |

### Determinism

All floating-point operations are deterministic. The integration uses a fixed step
size and no randomness. State sequences are compared with a tolerance of 1e-10.

### Friction Limit

The per-axle friction ellipse guarantees:

```
sqrt(Fxf^2 + Fyf^2) + sqrt(Fxr^2 + Fyr^2) <= mu * (Fzf + Fzr) = mu * m * g
```

by the triangle inequality. The total acceleration is therefore bounded by `mu * g`.

### Checkpoint/Recovery

Checkpoints are saved at step boundaries (post-blend state). Resuming from a
checkpoint continues the RK4 integration from the saved state. Since RK4 is
single-step and deterministic, the resumed trajectory matches the one-shot
trajectory exactly.

## REST API

| Method | Endpoint | Description |
|---|---|---|
| POST | `/api/scenarios` | Create a scenario |
| POST | `/api/scenarios/{id}/run` | Run a scenario |
| POST | `/api/scenarios/{id}/resume` | Resume from latest checkpoint |
| GET | `/api/scenarios/{id}/states?time=` | Query state at time |
| GET | `/api/scenarios/{id}` | Get scenario status |

## Configuration

The snapshot directory can be configured via the property
`simulator.snapshot-directory` (defaults to `${java.io.tmpdir}/vehicle-simulator-snapshots`).
