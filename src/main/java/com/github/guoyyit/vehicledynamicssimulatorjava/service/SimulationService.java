package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.checkpoint.CheckpointStore;
import com.github.guoyyit.vehicledynamicssimulatorjava.core.SimulationEngine;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.ScenarioStatus;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import org.springframework.stereotype.Service;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Orchestrates scenario runs. Runs are fully isolated (no shared mutable
 * state), so concurrent execution of the same or different scenarios is
 * deterministic and identical to sequential execution.
 */
@Service
public class SimulationService {

    private final SimulationEngine engine = new SimulationEngine();
    private final CheckpointStore checkpoints = new CheckpointStore();
    private final Map<String, ScenarioStatus> statusByRun = new ConcurrentHashMap<>();
    private final Map<String, Object> locksByRun = new ConcurrentHashMap<>();

    /**
     * Validates and runs a scenario to completion. On invalid input or
     * numerical divergence the run is marked FAILED and a
     * {@link com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException}
     * is raised; no partial result is returned.
     */
    public SimulationResult runScenario(String runId, Scenario scenario, SimulationConfig config) {
        validateOrFail(runId, scenario, config);
        synchronized (lockFor(runId)) {
            checkpoints.clear(runId);
            statusByRun.put(runId, ScenarioStatus.RUNNING);
            try {
                SimulationResult result = engine.run(runId, scenario, config, checkpoints);
                statusByRun.put(runId, ScenarioStatus.COMPLETED);
                return result;
            } catch (RuntimeException e) {
                statusByRun.put(runId, ScenarioStatus.FAILED);
                throw asSimulationException(e);
            }
        }
    }

    /**
     * Resumes a previously interrupted run from its latest checkpoint.
     */
    public SimulationResult resumeScenario(String runId, Scenario scenario, SimulationConfig config) {
        validateOrFail(runId, scenario, config);
        synchronized (lockFor(runId)) {
            checkpoints.latest(runId).ifPresent(state -> {
                if (state.time() > scenario.duration()) {
                    throw new SimulationException("checkpoint time " + state.time()
                            + " is beyond scenario duration " + scenario.duration());
                }
            });
            statusByRun.put(runId, ScenarioStatus.RUNNING);
            try {
                SimulationResult result = engine.resume(runId, scenario, config, checkpoints);
                statusByRun.put(runId, ScenarioStatus.COMPLETED);
                return result;
            } catch (RuntimeException e) {
                statusByRun.put(runId, ScenarioStatus.FAILED);
                throw asSimulationException(e);
            }
        }
    }

    /**
     * Runs a scenario from t=0 up to {@code stopTime} and stops, keeping the
     * checkpoints so a later {@link #resumeScenario} continues seamlessly.
     * Useful for chunked execution and for simulating an interrupted run.
     */
    public SimulationResult runScenarioUntil(String runId, Scenario scenario, SimulationConfig config,
                                             double stopTime) {
        validateOrFail(runId, scenario, config);
        requireFinite("stopTime", stopTime);
        if (stopTime <= 0.0 || stopTime > scenario.duration()) {
            statusByRun.put(runId, ScenarioStatus.FAILED);
            throw new SimulationException("stopTime must be in (0, duration], got " + stopTime);
        }
        synchronized (lockFor(runId)) {
            checkpoints.clear(runId);
            statusByRun.put(runId, ScenarioStatus.RUNNING);
            try {
                SimulationResult result = engine.runUntil(runId, scenario, config, checkpoints,
                        stopTime);
                statusByRun.put(runId, ScenarioStatus.RUNNING);
                return result;
            } catch (RuntimeException e) {
                statusByRun.put(runId, ScenarioStatus.FAILED);
                throw asSimulationException(e);
            }
        }
    }

    /**
     * Returns the latest known status of a run, or null if never started.
     */
    public ScenarioStatus statusOf(String runId) {
        return statusByRun.get(runId);
    }

    private Object lockFor(String runId) {
        return locksByRun.computeIfAbsent(runId, k -> new Object());
    }

    private void validateOrFail(String runId, Scenario scenario, SimulationConfig config) {
        try {
            validate(scenario, config);
        } catch (SimulationException e) {
            statusByRun.put(runId, ScenarioStatus.FAILED);
            throw e;
        }
    }

    private static SimulationException asSimulationException(RuntimeException e) {
        return e instanceof SimulationException se ? se
                : new SimulationException("simulation failed: " + e.getMessage(), e);
    }

    private static void validate(Scenario scenario, SimulationConfig config) {
        if (scenario == null) {
            throw new SimulationException("scenario must not be null");
        }
        if (scenario.scenarioId() == null || scenario.scenarioId().isBlank()) {
            throw new SimulationException("scenario id must not be blank");
        }
        validateVehicle(scenario.vehicle());
        requireFinite("frictionCoefficient", scenario.frictionCoefficient());
        if (scenario.frictionCoefficient() <= 0.0) {
            throw new SimulationException("frictionCoefficient must be positive, got "
                    + scenario.frictionCoefficient());
        }
        requireFinite("duration", scenario.duration());
        if (scenario.duration() <= 0.0) {
            throw new SimulationException("duration must be positive, got " + scenario.duration());
        }
        validateInputs(scenario);
        validateConfig(scenario, config);
    }

    private static void validateVehicle(VehicleParameters v) {
        if (v == null) {
            throw new SimulationException("vehicle parameters must not be null");
        }
        requirePositive("mass", v.mass());
        requirePositive("yawInertia", v.yawInertia());
        requirePositive("frontAxleToCg", v.frontAxleToCg());
        requirePositive("rearAxleToCg", v.rearAxleToCg());
        requirePositive("frontCorneringStiffness", v.frontCorneringStiffness());
        requirePositive("rearCorneringStiffness", v.rearCorneringStiffness());
        requirePositive("maxSteerAngle", v.maxSteerAngle());
        requireNonNegative("maxDriveAccel", v.maxDriveAccel());
        requireNonNegative("maxBrakeAccel", v.maxBrakeAccel());
    }

    private static void validateInputs(Scenario scenario) {
        if (scenario.inputs() == null) {
            throw new SimulationException("driving input schedule must not be null");
        }
        double previous = Double.NEGATIVE_INFINITY;
        for (DrivingInput input : scenario.inputs()) {
            requireFinite("input.time", input.time());
            requireFinite("input.longitudinalAccel", input.longitudinalAccel());
            requireFinite("input.steerAngle", input.steerAngle());
            if (input.time() < 0.0 || input.time() > scenario.duration()) {
                throw new SimulationException("input time " + input.time()
                        + " outside scenario interval [0, " + scenario.duration() + "]");
            }
            if (input.time() < previous) {
                throw new SimulationException("driving inputs must be sorted by time ascending");
            }
            previous = input.time();
        }
    }

    private static void validateConfig(Scenario scenario, SimulationConfig config) {
        if (config == null) {
            throw new SimulationException("simulation config must not be null");
        }
        requireFinite("stepSize", config.stepSize());
        if (config.stepSize() <= 0.0 || config.stepSize() > scenario.duration()) {
            throw new SimulationException("stepSize must be in (0, duration], got "
                    + config.stepSize());
        }
        if (config.checkpointEvery() < 1) {
            throw new SimulationException("checkpointEvery must be >= 1, got "
                    + config.checkpointEvery());
        }
        if (config.queryTimes() == null) {
            throw new SimulationException("queryTimes must not be null");
        }
        for (double t : config.queryTimes()) {
            requireFinite("queryTime", t);
            if (t < 0.0 || t > scenario.duration()) {
                throw new SimulationException("query time " + t
                        + " outside scenario interval [0, " + scenario.duration() + "]");
            }
        }
    }

    private static void requirePositive(String name, double value) {
        requireFinite(name, value);
        if (value <= 0.0) {
            throw new SimulationException(name + " must be positive, got " + value);
        }
    }

    private static void requireNonNegative(String name, double value) {
        requireFinite(name, value);
        if (value < 0.0) {
            throw new SimulationException(name + " must be non-negative, got " + value);
        }
    }

    private static void requireFinite(String name, double value) {
        if (!Double.isFinite(value)) {
            throw new SimulationException(name + " must be finite, got " + value);
        }
    }
}
