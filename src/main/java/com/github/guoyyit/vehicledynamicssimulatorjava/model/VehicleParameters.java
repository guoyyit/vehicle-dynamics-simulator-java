package com.github.guoyyit.vehicledynamicssimulatorjava.model;

/**
 * Immutable vehicle parameter set for a scenario.
 *
 * @param mass                 vehicle mass [kg]
 * @param yawInertia           yaw moment of inertia Iz [kg*m^2]
 * @param frontAxleToCg        distance from CG to front axle lf [m]
 * @param rearAxleToCg         distance from CG to rear axle lr [m]
 * @param frontCorneringStiffness front tire cornering stiffness Cf [N/rad]
 * @param rearCorneringStiffness  rear tire cornering stiffness Cr [N/rad]
 * @param maxSteerAngle        absolute steering angle limit [rad]
 * @param maxDriveAccel        maximum longitudinal acceleration command [m/s^2]
 * @param maxBrakeAccel        maximum longitudinal deceleration command [m/s^2], positive value
 */
public record VehicleParameters(
        double mass,
        double yawInertia,
        double frontAxleToCg,
        double rearAxleToCg,
        double frontCorneringStiffness,
        double rearCorneringStiffness,
        double maxSteerAngle,
        double maxDriveAccel,
        double maxBrakeAccel) {
}
