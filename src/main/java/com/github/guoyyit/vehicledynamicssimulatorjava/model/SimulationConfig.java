package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

import java.util.Arrays;

/**
 * Simulation configuration.
 */
public record SimulationConfig(
        double stepSize,
        double[] queryTimes,
        int checkpointIntervalSteps) {

    public SimulationConfig {
        if (Double.isNaN(stepSize) || Double.isInfinite(stepSize) || stepSize <= 0.0) {
            throw new ScenarioValidationException("stepSize must be finite and positive, got " + stepSize);
        }
        if (queryTimes == null || queryTimes.length == 0) {
            throw new ScenarioValidationException("queryTimes must not be empty");
        }
        for (double time : queryTimes) {
            if (Double.isNaN(time) || Double.isInfinite(time) || time < 0.0) {
                throw new ScenarioValidationException(
                        "queryTimes must be finite and non-negative, got " + time);
            }
        }
        if (checkpointIntervalSteps <= 0) {
            throw new ScenarioValidationException(
                    "checkpointIntervalSteps must be positive, got " + checkpointIntervalSteps);
        }
        double[] sorted = queryTimes.clone();
        Arrays.sort(sorted);
        queryTimes = sorted;
    }

    /**
     * Returns the last query time (the simulation horizon).
     */
    public double horizon() {
        return queryTimes[queryTimes.length - 1];
    }

    /**
     * Returns a defensive copy of the query times array.
     */
    @Override
    public double[] queryTimes() {
        return queryTimes.clone();
    }
}
