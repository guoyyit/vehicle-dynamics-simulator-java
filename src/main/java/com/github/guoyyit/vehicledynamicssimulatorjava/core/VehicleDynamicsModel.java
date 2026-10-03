package com.github.guoyyit.vehicledynamicssimulatorjava.core;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

/**
 * Dynamic bicycle model with coupled longitudinal/lateral dynamics and a
 * combined-slip friction ellipse constraint. Pure and deterministic: the
 * derivative depends only on (state, input, parameters, mu).
 */
public class VehicleDynamicsModel {

    /** Gravity constant [m/s^2]. */
    public static final double GRAVITY = 9.81;

    /**
     * Below this speed [m/s] lateral tire forces are faded out so that a
     * (nearly) stationary vehicle does not develop unphysical side forces.
     */
    public static final double LOW_SPEED_THRESHOLD = 0.5;

    private final VehicleParameters params;
    private final double frictionCoefficient;

    public VehicleDynamicsModel(VehicleParameters params, double frictionCoefficient) {
        this.params = params;
        this.frictionCoefficient = frictionCoefficient;
    }

    /**
     * Computes the time derivative of the dynamic state together with the
     * (friction-limited) accelerations for this instant.
     */
    public Derivative derivative(VehicleState state, DrivingInput input) {
        double vx = state.longitudinalVx();
        double vy = state.lateralVy();
        double yawRate = state.yawRate();
        double heading = state.heading();
        double steer = clamp(input.steerAngle(), -params.maxSteerAngle(), params.maxSteerAngle());

        double axCommand = clamp(input.longitudinalAccel(),
                -params.maxBrakeAccel(), params.maxDriveAccel());
        // Service brakes cannot push a stationary vehicle backwards.
        if (axCommand < 0.0 && vx <= 0.0) {
            axCommand = 0.0;
        }

        // Linear tire model on the bicycle axles, faded out at low speed.
        double speed = Math.hypot(vx, vy);
        double lowSpeedFactor = Math.min(1.0, speed / LOW_SPEED_THRESHOLD);
        double slipFront = Math.atan2(vy + params.frontAxleToCg() * yawRate, vx) - steer;
        double slipRear = Math.atan2(vy - params.rearAxleToCg() * yawRate, vx);
        double fyFront = -params.frontCorneringStiffness() * slipFront * lowSpeedFactor;
        double fyRear = -params.rearCorneringStiffness() * slipRear * lowSpeedFactor;
        double fx = params.mass() * axCommand;

        // Friction circle: the total tire force is scaled into mu * m * g.
        // Scaling (instead of independent clamping) keeps the force direction
        // and cannot oscillate, and caps combined acceleration at mu * g.
        double fyTotal = fyFront * Math.cos(steer) + fyRear;
        double forceTotal = Math.hypot(fx, fyTotal);
        double forceLimit = frictionCoefficient * params.mass() * GRAVITY;
        double scale = forceTotal > forceLimit ? forceLimit / forceTotal : 1.0;

        double fxLimited = fx * scale;
        double fyFrontLimited = fyFront * scale;
        double fyRearLimited = fyRear * scale;

        double ax = fxLimited / params.mass();
        double ay = (fyFrontLimited * Math.cos(steer) + fyRearLimited) / params.mass();

        double dVx = ax + yawRate * vy;
        double dVy = ay - yawRate * vx;
        double dYawRate = (params.frontAxleToCg() * fyFrontLimited * Math.cos(steer)
                - params.rearAxleToCg() * fyRearLimited) / params.yawInertia();

        double dX = vx * Math.cos(heading) - vy * Math.sin(heading);
        double dY = vx * Math.sin(heading) + vy * Math.cos(heading);

        return new Derivative(dX, dY, yawRate, dVx, dVy, dYawRate, ax, ay);
    }

    /**
     * Clamps a commanded acceleration vector to the friction ellipse
     * mu * g so the combined acceleration never exceeds the adhesion limit.
     */
    public double[] clampToFrictionLimit(double ax, double ay) {
        double limit = frictionCoefficient * GRAVITY;
        double magnitude = Math.hypot(ax, ay);
        if (magnitude <= limit || magnitude == 0.0) {
            return new double[]{ax, ay};
        }
        double scale = limit / magnitude;
        return new double[]{ax * scale, ay * scale};
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * State derivative in the dynamic state vector
     * (x, y, psi, vx, vy, r). Accelerations are carried separately in the
     * produced {@link VehicleState} snapshots.
     */
    public record Derivative(
            double dX,
            double dY,
            double dHeading,
            double dVx,
            double dVy,
            double dYawRate,
            double ax,
            double ay) {
    }
}
