package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

/**
 * Driver control inputs at a single point in time.
 */
public record DriverInput(
        double throttle,
        double brake,
        double steeringAngle) {

    /**
     * Validates throttle, brake and steering ranges.
     */
    public void validate() {
        if (Double.isNaN(throttle) || Double.isInfinite(throttle) || throttle < 0.0 || throttle > 1.0) {
            throw new ScenarioValidationException("Throttle must be in [0, 1], got " + throttle);
        }
        if (Double.isNaN(brake) || Double.isInfinite(brake) || brake < 0.0 || brake > 1.0) {
            throw new ScenarioValidationException("Brake must be in [0, 1], got " + brake);
        }
        if (Double.isNaN(steeringAngle) || Double.isInfinite(steeringAngle)) {
            throw new ScenarioValidationException("Steering angle must be finite, got " + steeringAngle);
        }
    }
}
