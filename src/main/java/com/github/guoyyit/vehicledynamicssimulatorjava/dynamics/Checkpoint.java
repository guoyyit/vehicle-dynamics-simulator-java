package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

/**
 * A saved simulation state at a step boundary.
 */
public record Checkpoint(
        String scenarioId,
        int stepIndex,
        double time,
        VehicleState state) {
}
