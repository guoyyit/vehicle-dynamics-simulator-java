package com.github.guoyyit.vehicledynamicssimulatorjava.model;

/**
 * Full dynamic state of the vehicle at one instant.
 *
 * @param time            time since scenario start [s]
 * @param x               inertial position x [m]
 * @param y               inertial position y [m]
 * @param heading         yaw angle psi [rad]
 * @param longitudinalVx  body-frame longitudinal velocity [m/s]
 * @param lateralVy       body-frame lateral velocity [m/s]
 * @param yawRate         yaw rate r [rad/s]
 * @param longitudinalAx  body-frame longitudinal acceleration [m/s^2]
 * @param lateralAy       body-frame lateral acceleration [m/s^2]
 */
public record VehicleState(
        double time,
        double x,
        double y,
        double heading,
        double longitudinalVx,
        double lateralVy,
        double yawRate,
        double longitudinalAx,
        double lateralAy) {

    public double speed() {
        return Math.hypot(longitudinalVx, lateralVy);
    }

    public double combinedAcceleration() {
        return Math.hypot(longitudinalAx, lateralAy);
    }
}
