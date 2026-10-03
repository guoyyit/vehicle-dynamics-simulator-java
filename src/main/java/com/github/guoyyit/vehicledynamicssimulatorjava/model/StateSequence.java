package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;

import java.util.List;

/**
 * Output of a simulation run.
 */
public record StateSequence(
        String scenarioId,
        List<VehicleState> stepStates,
        List<VehicleState> queryStates) {

    /**
     * Returns the state at the given time by interpolating between adjacent
     * step states. Returns the exact step state when the time coincides with a
     * step boundary.
     */
    public VehicleState stateAt(double time) {
        int last = stepStates.size() - 1;
        if (time < stepStates.get(0).time() - 1e-9 || time > stepStates.get(last).time() + 1e-9) {
            throw new SimulationException(
                    "Time " + time + " is outside the simulated horizon [0, "
                            + stepStates.get(last).time() + "]");
        }
        if (Math.abs(time - stepStates.get(0).time()) <= 1e-9) {
            return stepStates.get(0);
        }
        if (Math.abs(time - stepStates.get(last).time()) <= 1e-9) {
            return stepStates.get(last);
        }
        for (int i = 0; i < last; i++) {
            VehicleState a = stepStates.get(i);
            VehicleState b = stepStates.get(i + 1);
            if (time >= a.time() - 1e-9 && time <= b.time() + 1e-9) {
                if (Math.abs(time - a.time()) <= 1e-9) {
                    return a;
                }
                if (Math.abs(time - b.time()) <= 1e-9) {
                    return b;
                }
                double fraction = (time - a.time()) / (b.time() - a.time());
                return VehicleState.interpolate(a, b, fraction);
            }
        }
        throw new SimulationException("Time " + time + " is outside the simulated horizon");
    }
}
