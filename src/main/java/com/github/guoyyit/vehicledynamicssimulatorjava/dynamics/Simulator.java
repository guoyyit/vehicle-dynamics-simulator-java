package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.DivergenceException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-step deterministic simulator using a fourth-order Runge-Kutta
 * integrator. All state is kept at step boundaries so that resuming from a
 * checkpoint reproduces the same trajectory bit-for-bit.
 */
public final class Simulator {

    private static final double MAX_SPEED = 200.0;
    private static final double TIME_EPS = 1e-9;

    private final Scenario scenario;
    private final SimulationConfig config;
    private final CheckpointCallback callback;
    private final double[] initialState;
    private int initialStep;
    private double initialTime;

    public Simulator(Scenario scenario, SimulationConfig config, CheckpointCallback callback) {
        this.scenario = scenario;
        this.config = config;
        this.callback = callback;
        this.initialState = new double[6];
        this.initialStep = 0;
        this.initialTime = 0.0;
    }

    public static Simulator resume(
            Scenario scenario,
            SimulationConfig config,
            Checkpoint checkpoint,
            CheckpointCallback callback) {
        Simulator simulator = new Simulator(scenario, config, callback);
        VehicleState state = checkpoint.state();
        simulator.initialState[VehicleDynamics.VX] = state.vx();
        simulator.initialState[VehicleDynamics.VY] = state.vy();
        simulator.initialState[VehicleDynamics.YAW_RATE] = state.yawRate();
        simulator.initialState[VehicleDynamics.YAW] = state.yaw();
        simulator.initialState[VehicleDynamics.X] = state.x();
        simulator.initialState[VehicleDynamics.Y] = state.y();
        simulator.initialStep = checkpoint.stepIndex();
        simulator.initialTime = checkpoint.time();
        return simulator;
    }

    /**
     * Runs the simulation from the initial (or resumed) state to the horizon.
     */
    public StateSequence run() {
        double dt = config.stepSize();
        double horizon = config.horizon();

        List<VehicleState> stepStates = new ArrayList<>();

        double[] y = initialState.clone();
        double t = initialTime;
        int step = initialStep;

        VehicleState current = toState(t, y, 0.0, 0.0);
        current = recomputeAcceleration(current);
        stepStates.add(current);
        notifyCheckpoint(step, t, current);

        while (t < horizon - TIME_EPS) {
            double stepDt = Math.min(dt, horizon - t);

            double[] k1 = derivativesAt(t, y);
            double[] y2 = addScaled(y, k1, stepDt / 2.0);
            double[] k2 = derivativesAt(t + stepDt / 2.0, y2);
            double[] y3 = addScaled(y, k2, stepDt / 2.0);
            double[] k3 = derivativesAt(t + stepDt / 2.0, y3);
            double[] y4 = addScaled(y, k3, stepDt);
            double[] k4 = derivativesAt(t + stepDt, y4);

            for (int i = 0; i < 6; i++) {
                y[i] += stepDt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }

            step++;
            t = (step - initialStep) * dt + initialTime;

            VehicleState stateAtT = toState(t, y, 0.0, 0.0);
            stateAtT = VehicleDynamics.applyLowSpeedBlend(
                    stateAtT, scenario.schedule().inputAt(t).steeringAngle(), scenario.vehicle());
            stateAtT = recomputeAcceleration(stateAtT);
            checkBounds(stateAtT);

            y[VehicleDynamics.VX] = stateAtT.vx();
            y[VehicleDynamics.VY] = stateAtT.vy();
            y[VehicleDynamics.YAW_RATE] = stateAtT.yawRate();
            y[VehicleDynamics.YAW] = stateAtT.yaw();
            y[VehicleDynamics.X] = stateAtT.x();
            y[VehicleDynamics.Y] = stateAtT.y();

            stepStates.add(stateAtT);
            notifyCheckpoint(step, t, stateAtT);
        }

        List<VehicleState> queryStates = queryStates(stepStates);
        return new StateSequence(scenario.id(), stepStates, queryStates);
    }

    private double[] derivativesAt(double time, double[] state) {
        VehicleState vehicleState = toState(time, state, 0.0, 0.0);
        return VehicleDynamics.compute(
                scenario.vehicle(), scenario.road(),
                scenario.schedule().inputAt(time), vehicleState).derivatives();
    }

    private VehicleState recomputeAcceleration(VehicleState state) {
        VehicleDynamics.DynamicsResult result = VehicleDynamics.compute(
                scenario.vehicle(), scenario.road(),
                scenario.schedule().inputAt(state.time()), state);
        return new VehicleState(
                state.time(), state.x(), state.y(), state.vx(), state.vy(),
                result.ax(), result.ay(), state.yaw(), state.yawRate());
    }

    private List<VehicleState> queryStates(List<VehicleState> stepStates) {
        double[] times = config.queryTimes();
        List<VehicleState> result = new ArrayList<>(times.length);
        for (double queryTime : times) {
            result.add(queryState(stepStates, queryTime));
        }
        return result;
    }

    private VehicleState queryState(List<VehicleState> stepStates, double time) {
        int last = stepStates.size() - 1;
        if (time <= stepStates.get(0).time() + TIME_EPS) {
            return stepStates.get(0);
        }
        if (time >= stepStates.get(last).time() - TIME_EPS) {
            return stepStates.get(last);
        }
        for (int i = 0; i < last; i++) {
            VehicleState a = stepStates.get(i);
            VehicleState b = stepStates.get(i + 1);
            if (time >= a.time() - TIME_EPS && time <= b.time() + TIME_EPS) {
                if (Math.abs(time - a.time()) <= TIME_EPS) {
                    return a;
                }
                if (Math.abs(time - b.time()) <= TIME_EPS) {
                    return b;
                }
                double fraction = (time - a.time()) / (b.time() - a.time());
                return VehicleState.interpolate(a, b, fraction);
            }
        }
        throw new DivergenceException("Query time " + time + " is outside the simulated horizon");
    }

    private void notifyCheckpoint(int step, double time, VehicleState state) {
        if (callback != null && step % config.checkpointIntervalSteps() == 0) {
            callback.onCheckpoint(step, time, state);
        }
    }

    private static double[] addScaled(double[] a, double[] b, double scale) {
        double[] result = a.clone();
        for (int i = 0; i < result.length; i++) {
            result[i] += scale * b[i];
        }
        return result;
    }

    private static VehicleState toState(double time, double[] y, double ax, double ay) {
        return new VehicleState(time, y[VehicleDynamics.X], y[VehicleDynamics.Y],
                y[VehicleDynamics.VX], y[VehicleDynamics.VY], ax, ay,
                y[VehicleDynamics.YAW], y[VehicleDynamics.YAW_RATE]);
    }

    private static void checkBounds(VehicleState state) {
        if (Math.abs(state.vx()) > MAX_SPEED || Math.abs(state.vy()) > MAX_SPEED) {
            throw new DivergenceException(
                    "Speed out of bounds: vx=" + state.vx() + ", vy=" + state.vy());
        }
        if (!Double.isFinite(state.x()) || !Double.isFinite(state.y())
                || !Double.isFinite(state.yaw()) || !Double.isFinite(state.yawRate())) {
            throw new DivergenceException("Numerical divergence detected: non-finite state");
        }
    }
}
