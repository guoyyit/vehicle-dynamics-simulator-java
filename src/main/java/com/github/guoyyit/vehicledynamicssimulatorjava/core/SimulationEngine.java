package com.github.guoyyit.vehicledynamicssimulatorjava.core;

import com.github.guoyyit.vehicledynamicssimulatorjava.checkpoint.CheckpointStore;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-step deterministic integrator. RK4 with a zero-order-hold input
 * schedule; identical (scenario, config) always yields identical sequences.
 *
 * <p>The time grid is defined by step index: state k sits exactly at
 * {@code k * stepSize}. If the duration is not a multiple of the step size,
 * one final partial step lands exactly on the duration. Checkpoints therefore
 * always lie on the same grid and resuming reproduces the uninterrupted
 * sequence bit for bit.
 */
public class SimulationEngine {

    private static final double TIME_TOLERANCE = 1e-9;

    private final StateSampler sampler = new StateSampler();

    /**
     * Runs a scenario from t=0 to its duration.
     */
    public SimulationResult run(String runId, Scenario scenario, SimulationConfig config,
                                CheckpointStore checkpoints) {
        VehicleDynamicsModel model = new VehicleDynamicsModel(scenario.vehicle(),
                scenario.frictionCoefficient());
        VehicleState initial = initialState(scenario, model);
        List<VehicleState> states = integrate(runId, scenario, config, checkpoints, model, initial,
                scenario.duration());
        return new SimulationResult(scenario.scenarioId(), states, sampleQueries(states, config));
    }

    /**
     * Integrates from t=0 up to {@code stopTime} and stops, persisting
     * checkpoints. Models an interrupted run: the stored checkpoints allow a
     * later {@link #resume} to continue exactly where this run stopped.
     */
    public SimulationResult runUntil(String runId, Scenario scenario, SimulationConfig config,
                                     CheckpointStore checkpoints, double stopTime) {
        VehicleDynamicsModel model = new VehicleDynamicsModel(scenario.vehicle(),
                scenario.frictionCoefficient());
        VehicleState initial = initialState(scenario, model);
        List<VehicleState> states = integrate(runId, scenario, config, checkpoints, model, initial,
                stopTime);
        return new SimulationResult(scenario.scenarioId(), states, sampleQueries(states, config));
    }

    /**
     * Resumes a scenario from the latest saved checkpoint. The produced
     * sequence is point-wise identical to an uninterrupted run.
     */
    public SimulationResult resume(String runId, Scenario scenario, SimulationConfig config,
                                   CheckpointStore checkpoints) {
        VehicleDynamicsModel model = new VehicleDynamicsModel(scenario.vehicle(),
                scenario.frictionCoefficient());
        VehicleState from = checkpoints.latest(runId).orElseGet(() -> initialState(scenario, model));
        List<VehicleState> states = integrate(runId, scenario, config, checkpoints, model, from,
                scenario.duration());
        return new SimulationResult(scenario.scenarioId(), states, sampleQueries(states, config));
    }

    /**
     * Integrates from {@code from} (inclusive) to {@code toTime},
     * persisting checkpoints every {@code checkpointEvery} steps.
     */
    private List<VehicleState> integrate(String runId, Scenario scenario, SimulationConfig config,
                                         CheckpointStore checkpoints, VehicleDynamicsModel model,
                                         VehicleState from, double toTime) {
        double dt = config.stepSize();
        int startIndex = (int) Math.round(from.time() / dt);
        int fullSteps = (int) Math.floor(toTime / dt + TIME_TOLERANCE);
        double remainder = toTime - fullSteps * dt;

        List<VehicleState> states = new ArrayList<>();
        states.add(from);
        checkpoints.save(runId, from);

        VehicleState current = from;
        int index = startIndex;
        int inputIndex = inputIndexAt(scenario.inputs(), -1, current.time());
        while (index < fullSteps) {
            inputIndex = inputIndexAt(scenario.inputs(), inputIndex, current.time());
            DrivingInput input = inputAt(scenario.inputs(), inputIndex);
            current = step(current, input, model, dt, (index + 1) * dt);
            current = stampAccelerations(current, scenario, model);
            checkState(current, scenario);
            states.add(current);
            index++;
            if (index % config.checkpointEvery() == 0) {
                checkpoints.save(runId, current);
            }
        }
        if (remainder > TIME_TOLERANCE && current.time() < toTime - TIME_TOLERANCE) {
            inputIndex = inputIndexAt(scenario.inputs(), inputIndex, current.time());
            DrivingInput input = inputAt(scenario.inputs(), inputIndex);
            current = step(current, input, model, remainder, toTime);
            current = stampAccelerations(current, scenario, model);
            checkState(current, scenario);
            states.add(current);
            checkpoints.save(runId, current);
        }
        return states;
    }

    /**
     * Integrates one RK4 step from {@code state} with constant input,
     * stamping the result with the exact grid time {@code newTime}.
     */
    VehicleState step(VehicleState state, DrivingInput input, VehicleDynamicsModel model,
                      double dt, double newTime) {
        VehicleDynamicsModel.Derivative k1 = model.derivative(state, input);
        VehicleDynamicsModel.Derivative k2 = model.derivative(advance(state, k1, dt * 0.5), input);
        VehicleDynamicsModel.Derivative k3 = model.derivative(advance(state, k2, dt * 0.5), input);
        VehicleDynamicsModel.Derivative k4 = model.derivative(advance(state, k3, dt), input);

        double x = state.x() + dt / 6.0 * (k1.dX() + 2.0 * k2.dX() + 2.0 * k3.dX() + k4.dX());
        double y = state.y() + dt / 6.0 * (k1.dY() + 2.0 * k2.dY() + 2.0 * k3.dY() + k4.dY());
        double heading = state.heading()
                + dt / 6.0 * (k1.dHeading() + 2.0 * k2.dHeading() + 2.0 * k3.dHeading() + k4.dHeading());
        double vx = state.longitudinalVx()
                + dt / 6.0 * (k1.dVx() + 2.0 * k2.dVx() + 2.0 * k3.dVx() + k4.dVx());
        double vy = state.lateralVy()
                + dt / 6.0 * (k1.dVy() + 2.0 * k2.dVy() + 2.0 * k3.dVy() + k4.dVy());
        double yawRate = state.yawRate()
                + dt / 6.0 * (k1.dYawRate() + 2.0 * k2.dYawRate() + 2.0 * k3.dYawRate() + k4.dYawRate());
        return new VehicleState(newTime, x, y, heading, vx, vy, yawRate, 0.0, 0.0);
    }

    private static VehicleState advance(VehicleState state, VehicleDynamicsModel.Derivative d, double dt) {
        return new VehicleState(
                state.time() + dt,
                state.x() + d.dX() * dt,
                state.y() + d.dY() * dt,
                state.heading() + d.dHeading() * dt,
                state.longitudinalVx() + d.dVx() * dt,
                state.lateralVy() + d.dVy() * dt,
                state.yawRate() + d.dYawRate() * dt,
                0.0, 0.0);
    }

    private VehicleState initialState(Scenario scenario, VehicleDynamicsModel model) {
        VehicleState zero = new VehicleState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        return stampAccelerations(zero, scenario, model);
    }

    /**
     * Evaluates the friction-limited accelerations at {@code state} with the
     * input active at that instant and returns a stamped copy.
     */
    private VehicleState stampAccelerations(VehicleState state, Scenario scenario,
                                            VehicleDynamicsModel model) {
        DrivingInput input = inputAt(scenario.inputs(),
                inputIndexAt(scenario.inputs(), -1, state.time()));
        VehicleDynamicsModel.Derivative d = model.derivative(state, input);
        return new VehicleState(state.time(), state.x(), state.y(), state.heading(),
                state.longitudinalVx(), state.lateralVy(), state.yawRate(), d.ax(), d.ay());
    }

    private List<VehicleState> sampleQueries(List<VehicleState> states, SimulationConfig config) {
        List<VehicleState> sampled = new ArrayList<>();
        for (double t : config.queryTimes()) {
            sampled.add(sampler.sample(states, t));
        }
        return sampled;
    }

    private static DrivingInput inputAt(List<DrivingInput> inputs, int index) {
        return index >= 0 ? inputs.get(index) : new DrivingInput(0.0, 0.0, 0.0);
    }

    /**
     * Zero-order hold: returns the index of the latest sample with
     * {@code time <= t}, scanning forward from {@code from} (-1 = none yet).
     */
    private static int inputIndexAt(List<DrivingInput> inputs, int from, double t) {
        int i = from;
        while (i + 1 < inputs.size() && inputs.get(i + 1).time() <= t) {
            i++;
        }
        return i;
    }

    private static void checkState(VehicleState state, Scenario scenario) {
        double[] values = {state.time(), state.x(), state.y(), state.heading(),
                state.longitudinalVx(), state.lateralVy(), state.yawRate(),
                state.longitudinalAx(), state.lateralAy()};
        for (double v : values) {
            if (!Double.isFinite(v)) {
                throw new SimulationException("numerical divergence in scenario '"
                        + scenario.scenarioId() + "' at t=" + state.time());
            }
        }
        double limit = scenario.frictionCoefficient() * VehicleDynamicsModel.GRAVITY;
        if (state.combinedAcceleration() > limit * (1.0 + 1e-9) + 1e-9) {
            throw new SimulationException("combined acceleration " + state.combinedAcceleration()
                    + " exceeds friction limit " + limit + " in scenario '"
                    + scenario.scenarioId() + "' at t=" + state.time());
        }
    }
}
