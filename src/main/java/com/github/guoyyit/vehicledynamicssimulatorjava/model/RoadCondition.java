package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

/**
 * Road surface conditions.
 */
public record RoadCondition(
        double frictionCoefficient,
        double rollingResistanceCoefficient) {

    /**
     * Validates road condition ranges.
     */
    public void validate() {
        if (Double.isNaN(frictionCoefficient) || Double.isInfinite(frictionCoefficient)
                || frictionCoefficient <= 0.0 || frictionCoefficient > 1.0) {
            throw new ScenarioValidationException(
                    "frictionCoefficient must be in (0, 1], got " + frictionCoefficient);
        }
        if (Double.isNaN(rollingResistanceCoefficient) || Double.isInfinite(rollingResistanceCoefficient)
                || rollingResistanceCoefficient < 0.0 || rollingResistanceCoefficient >= 1.0) {
            throw new ScenarioValidationException(
                    "rollingResistanceCoefficient must be in [0, 1), got " + rollingResistanceCoefficient);
        }
    }
}
