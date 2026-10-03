package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

import java.util.List;

/**
 * Time-indexed driver inputs.
 */
public record DrivingSchedule(List<TimedInput> inputs) {

    public DrivingSchedule {
        if (inputs == null || inputs.isEmpty()) {
            throw new ScenarioValidationException("Driving schedule must not be empty");
        }
        double previous = -1.0;
        for (TimedInput timed : inputs) {
            if (timed == null || timed.input() == null) {
                throw new ScenarioValidationException("Schedule entries must not be null");
            }
            double time = timed.time();
            if (Double.isNaN(time) || Double.isInfinite(time) || time < 0.0) {
                throw new ScenarioValidationException(
                        "Schedule times must be finite and non-negative, got " + time);
            }
            if (time <= previous) {
                throw new ScenarioValidationException("Schedule times must be strictly increasing");
            }
            previous = time;
            timed.input().validate();
        }
    }

    /**
     * Linearly interpolates the driver input at the given time. Inputs before
     * the first entry or after the last entry are held at the nearest value.
     */
    public DriverInput inputAt(double time) {
        int index = 0;
        while (index < inputs.size() - 1 && inputs.get(index + 1).time() <= time) {
            index++;
        }
        TimedInput left = inputs.get(index);
        TimedInput right = inputs.get(Math.min(index + 1, inputs.size() - 1));
        if (left.time() == right.time()) {
            return left.input();
        }
        double fraction = (time - left.time()) / (right.time() - left.time());
        if (fraction <= 0.0) {
            return left.input();
        }
        if (fraction >= 1.0) {
            return right.input();
        }
        DriverInput a = left.input();
        DriverInput b = right.input();
        return new DriverInput(
                a.throttle() + fraction * (b.throttle() - a.throttle()),
                a.brake() + fraction * (b.brake() - a.brake()),
                a.steeringAngle() + fraction * (b.steeringAngle() - a.steeringAngle()));
    }

    /**
     * One driver input at a given simulation time.
     */
    public record TimedInput(double time, DriverInput input) {
    }
}
