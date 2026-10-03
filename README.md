# Vehicle Dynamics Simulator (Java / Spring Boot)

Deterministic vehicle dynamics simulation service. It receives vehicle
parameters and driving inputs, propagates the vehicle motion, and serves
closed-loop verification for upper-layer control and planning logic, with
deterministic replay and failure recovery.

## Model Assumptions

- **Dynamic bicycle model**: the two axles are lumped into single front/rear
  wheels. State vector: inertial position `(x, y)`, heading `psi`, body-frame
  velocities `(vx, vy)`, yaw rate `r`.
- **Linear tire model**: lateral tire force `Fy = -C * alpha` per axle, with
  slip angles from the body-frame velocities. No longitudinal load transfer;
  total normal load is constant `m * g`.
- **Friction circle**: the combined tire force is capped at `mu * m * g`.
  When demand exceeds the limit, the whole force vector is scaled
  proportionally (direction preserved), so the combined acceleration never
  exceeds `mu * g` and cannot oscillate or diverge at the limit.
- **Low-speed regularization**: below 0.5 m/s lateral tire forces fade out
  linearly, so a (nearly) stationary vehicle does not develop unphysical side
  forces; steering only takes effect once the vehicle moves.
- **Actuator limits**: steering angle is clamped to `maxSteerAngle`,
  longitudinal command to `[-maxBrakeAccel, maxDriveAccel]`; service brakes
  cannot push a stationary vehicle backwards.
- **Zero-order hold inputs**: each driving input sample applies from its
  timestamp until the next sample. Samples before the first entry are zero.
- **Fixed-step RK4 integration** with step size `dt`. The time grid is
  `t_k = k * dt`; if the duration is not a multiple of `dt`, one final
  partial step lands exactly on the duration. No randomness, no wall-clock
  dependence: identical scenario + config reproduces the sequence bit for
  bit.

## Parameters

### VehicleParameters
| Field | Unit | Meaning |
| --- | --- | --- |
| `mass` | kg | vehicle mass |
| `yawInertia` | kg*m^2 | yaw moment of inertia Iz |
| `frontAxleToCg` / `rearAxleToCg` | m | CG-to-axle distances lf / lr |
| `frontCorneringStiffness` / `rearCorneringStiffness` | N/rad | tire cornering stiffness Cf / Cr |
| `maxSteerAngle` | rad | absolute steering limit |
| `maxDriveAccel` / `maxBrakeAccel` | m/s^2 | longitudinal actuator limits |

### Scenario
| Field | Meaning |
| --- | --- |
| `scenarioId` | unique scenario identifier |
| `vehicle` | vehicle parameter set |
| `frictionCoefficient` | road-tire friction coefficient mu |
| `inputs` | driving input schedule `(time, longitudinalAccel, steerAngle)`, sorted by time |
| `duration` | simulated duration [s] |

### SimulationConfig
| Field | Meaning |
| --- | --- |
| `stepSize` | fixed integration step dt [s] |
| `queryTimes` | extra time points sampled by deterministic linear interpolation |
| `checkpointEvery` | persist a recovery checkpoint every N steps |

### Output (VehicleState)
`time`, `x`, `y`, `heading`, body-frame `longitudinalVx` / `lateralVy`,
`yawRate`, and friction-limited body-frame accelerations `longitudinalAx` /
`lateralAy`. The combined acceleration `hypot(ax, ay)` never exceeds
`mu * g`.

## API

- `POST /api/simulation/{runId}/run` — validate and run a scenario to
  completion. Body: `{"scenario": ..., "config": ...}`.
- `POST /api/simulation/{runId}/resume` — continue from the latest
  checkpoint of `runId`; the continuation is point-wise identical to an
  uninterrupted run.
- Errors (invalid input, out-of-range parameters, divergence) are reported
  as `422` with an explicit message; the run is marked `FAILED` and no
  partial result is returned.

`SimulationService` additionally exposes `runScenarioUntil(...)` for chunked
execution (used to simulate interruptions) and `statusOf(runId)`.

## Determinism, Recovery, Concurrency

- **Replay**: same scenario + config => identical state sequence, verified
  by exact (bit-wise) equality of `VehicleState` records.
- **Recovery**: checkpoints are stored per `runId` every
  `checkpointEvery` steps. `resume` continues from the latest checkpoint on
  the same time grid (`t_k = k * dt`), so stitched and one-shot sequences
  are identical.
- **Concurrency**: the engine is stateless; checkpoints and statuses are
  keyed by `runId` in concurrent maps, and each `runId` is serialized by a
  per-run lock. Concurrent runs of the same or different scenarios match
  sequential execution exactly.

## Verification

```
mvn test
```

| Test class | Covers |
| --- | --- |
| `DeterminismTest` | point-wise reproducibility; required output channels; low/high-mu behavior differences |
| `FrictionLimitTest` | combined acceleration <= mu*g at every step; saturation stability without oscillation |
| `QueryConsistencyTest` | exact results on step times; interpolation between steps; deterministic sampling; range checks |
| `RecoveryTest` | resume equals one-shot tail bit for bit; checkpoint spacing; stitching |
| `ConcurrencyTest` | concurrent repeats equal sequential run; multi-scenario isolation |
| `ValidationTest` | invalid parameters/config/schedule rejected; divergence raises explicit error |
