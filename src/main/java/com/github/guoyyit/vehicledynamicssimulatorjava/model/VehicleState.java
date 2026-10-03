package com.github.guoyyit.vehicledynamicssimulatorjava.model;

/**
 * Complete vehicle state at a point in time.
 */
public record VehicleState(
        double time,
        double x,
        double y,
        double vx,
        double vy,
        double ax,
        double ay,
        double yaw,
        double yawRate) {

    /**
     * Linearly interpolates between two states. The yaw angle is unwrapped so
     * that the interpolation follows the shortest angular path.
     */
    public static VehicleState interpolate(VehicleState a, VehicleState b, double fraction) {
        double f = Math.max(0.0, Math.min(1.0, fraction));
        double deltaYaw = b.yaw() - a.yaw();
        while (deltaYaw > Math.PI) {
            deltaYaw -= 2.0 * Math.PI;
        }
        while (deltaYaw < -Math.PI) {
            deltaYaw += 2.0 * Math.PI;
        }
        return new VehicleState(
                a.time() + f * (b.time() - a.time()),
                a.x() + f * (b.x() - a.x()),
                a.y() + f * (b.y() - a.y()),
                a.vx() + f * (b.vx() - a.vx()),
                a.vy() + f * (b.vy() - a.vy()),
                a.ax() + f * (b.ax() - a.ax()),
                a.ay() + f * (b.ay() - a.ay()),
                a.yaw() + f * deltaYaw,
                a.yawRate() + f * (b.yawRate() - a.yawRate()));
    }
}
