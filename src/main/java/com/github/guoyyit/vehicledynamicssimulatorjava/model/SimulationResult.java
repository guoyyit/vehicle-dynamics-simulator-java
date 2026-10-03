package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import java.util.List;

/**
 * Output of one scenario run.
 *
 * @param scenarioId scenario identifier
 * @param states     state sequence at integration step times
 * @param queried    states sampled at configured query times
 */
public record SimulationResult(String scenarioId, List<VehicleState> states, List<VehicleState> queried) {
}
