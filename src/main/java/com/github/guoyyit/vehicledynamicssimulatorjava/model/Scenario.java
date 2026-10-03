package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import java.util.List;

/**
 * A simulation scenario: vehicle parameters, road friction and the driving
 * input schedule. Identical scenario + config must reproduce identical state
 * sequences.
 *
 * @param scenarioId       unique scenario identifier
 * @param vehicle          vehicle parameter set
 * @param frictionCoefficient road-tire friction coefficient mu [-]
 * @param inputs           driving input schedule, sorted by time ascending
 * @param duration         total simulated duration [s]
 */
public record Scenario(
        String scenarioId,
        VehicleParameters vehicle,
        double frictionCoefficient,
        List<DrivingInput> inputs,
        double duration) {
}
