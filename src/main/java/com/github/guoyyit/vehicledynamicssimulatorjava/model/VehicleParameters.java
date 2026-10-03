package com.github.guoyyit.vehicledynamicssimulatorjava.model;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;

/**
 * Physical parameters of the vehicle (SI units).
 */
public record VehicleParameters(
        double mass,
        double wheelbase,
        double frontAxleDistance,
        double rearAxleDistance,
        double cgHeight,
        double yawInertia,
        double frontCorneringStiffness,
        double rearCorneringStiffness,
        double maxSteerAngle,
        double maxDriveForce,
        double maxBrakeForce) {

    /**
     * Validates physical parameter ranges and consistency.
     */
    public void validate() {
        requirePositive(mass, "mass");
        requirePositive(wheelbase, "wheelbase");
        requirePositive(frontAxleDistance, "frontAxleDistance");
        requirePositive(rearAxleDistance, "rearAxleDistance");
        requirePositive(yawInertia, "yawInertia");
        requirePositive(frontCorneringStiffness, "frontCorneringStiffness");
        requirePositive(rearCorneringStiffness, "rearCorneringStiffness");
        requirePositive(maxSteerAngle, "maxSteerAngle");
        requirePositive(maxDriveForce, "maxDriveForce");
        requirePositive(maxBrakeForce, "maxBrakeForce");
        if (Double.isNaN(cgHeight) || Double.isInfinite(cgHeight) || cgHeight < 0.0) {
            throw new ScenarioValidationException("cgHeight must be finite and non-negative, got " + cgHeight);
        }
        double axleSum = frontAxleDistance + rearAxleDistance;
        if (Math.abs(axleSum - wheelbase) > 1e-6 * wheelbase) {
            throw new ScenarioValidationException(
                    "frontAxleDistance + rearAxleDistance must equal wheelbase: "
                            + axleSum + " vs " + wheelbase);
        }
        if (cgHeight >= wheelbase) {
            throw new ScenarioValidationException(
                    "cgHeight must be smaller than wheelbase, got " + cgHeight);
        }
    }

    private static void requirePositive(double value, String name) {
        if (Double.isNaN(value) || Double.isInfinite(value) || value <= 0.0) {
            throw new ScenarioValidationException(name + " must be finite and positive, got " + value);
        }
    }
}
