package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import java.util.List;

/**
 * Execution configuration for a scenario run.
 *
 * @param stepSize        fixed integration step [s]
 * @param queryTimes      additional time points at which states are sampled
 * @param checkpointEvery persist a checkpoint every N integration steps
 */
public record SimulationConfig(double stepSize, List<Double> queryTimes, int checkpointEvery) {

    public static SimulationConfig of(double stepSize) {
        return new SimulationConfig(stepSize, List.of(), 1);
    }
}
