package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

/**
 * A simulation scenario: vehicle parameters, road conditions and driving schedule.
 */
public record Scenario(
        String id,
        VehicleParameters vehicle,
        RoadCondition road,
        DrivingSchedule schedule) {

    /**
     * Validates the scenario and cross-checks driver inputs against vehicle limits.
     */
    public void validate() {
        if (id == null || id.isBlank()) {
            throw new ScenarioValidationException("Scenario id must not be blank");
        }
        if (vehicle == null) {
            throw new ScenarioValidationException("Vehicle parameters must not be null");
        }
        if (road == null) {
            throw new ScenarioValidationException("Road condition must not be null");
        }
        if (schedule == null) {
            throw new ScenarioValidationException("Driving schedule must not be null");
        }
        vehicle.validate();
        road.validate();
        for (DrivingSchedule.TimedInput timed : schedule.inputs()) {
            double steer = timed.input().steeringAngle();
            if (Math.abs(steer) > vehicle.maxSteerAngle()) {
                throw new ScenarioValidationException(
                        "Steering angle " + steer + " at time " + timed.time()
                                + " exceeds vehicle max steer angle " + vehicle.maxSteerAngle());
            }
        }
    }
}
