package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.DivergenceException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DriverInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

/**
 * Planar single-track (bicycle) vehicle dynamics model.
 *
 * <p>Model assumptions:
 * <ul>
 *   <li>Body-fixed coordinate system: x forward, y left, yaw counter-clockwise.</li>
 *   <li>Linear tire model: lateral force = cornering stiffness * slip angle.</li>
 *   <li>Friction ellipse per axle caps the combined longitudinal and lateral
 *       force to {@code mu * Fz}, which guarantees the total acceleration stays
 *       within the tire-road adhesion limit.</li>
 *   <li>Longitudinal load transfer is quasi-static (based on commanded
 *       longitudinal acceleration).</li>
 *   <li>At low speed the dynamic model is blended toward a kinematic bicycle
 *       model to avoid slip-angle singularities and numerical divergence.</li>
 *   <li>Rear-wheel drive; braking force is split 60% front / 40% rear.</li>
 * </ul>
 */
public final class VehicleDynamics {

    /**
     * Internal dynamic state ordering: [vx, vy, yawRate, yaw, x, y].
     */
    public static final int VX = 0;
    public static final int VY = 1;
    public static final int YAW_RATE = 2;
    public static final int YAW = 3;
    public static final int X = 4;
    public static final int Y = 5;

    private static final double GRAVITY = 9.81;
    private static final double SPEED_EPS = 0.5;
    private static final double LOW_SPEED = 1.0;
    private static final double HIGH_SPEED = 4.0;
    private static final double BRAKE_FRONT_FRACTION = 0.6;

    private VehicleDynamics() {
    }

    /**
     * Result of a dynamics evaluation: state derivatives plus the actual
     * longitudinal and lateral accelerations.
     */
    public record DynamicsResult(double[] derivatives, double ax, double ay) {
    }

    /**
     * Computes the dynamics at the given state.
     */
    public static DynamicsResult compute(
            VehicleParameters vehicle,
            RoadCondition road,
            DriverInput input,
            VehicleState state) {
        double vx = state.vx();
        double vy = state.vy();
        double r = state.yawRate();
        double delta = input.steeringAngle();

        double driveForce = input.throttle() * vehicle.maxDriveForce();
        double brakeCommand = input.brake() * vehicle.maxBrakeForce();
        double brakeForce = brakeCommand * Math.tanh(vx / SPEED_EPS);
        double fxCommand = driveForce - brakeForce;
        double rollingResistance = road.rollingResistanceCoefficient() * vehicle.mass() * GRAVITY;
        if (Math.abs(vx) > SPEED_EPS) {
            fxCommand -= Math.signum(vx) * rollingResistance;
        }

        double mass = vehicle.mass();
        double wheelbase = vehicle.wheelbase();
        double lf = vehicle.frontAxleDistance();
        double lr = vehicle.rearAxleDistance();
        double h = vehicle.cgHeight();

        double axCommand = fxCommand / mass;
        double fzFront = (mass * GRAVITY * lr - mass * axCommand * h) / wheelbase;
        double fzRear = (mass * GRAVITY * lf + mass * axCommand * h) / wheelbase;
        if (fzFront < 0.0) {
            fzFront = 0.0;
        }
        if (fzRear < 0.0) {
            fzRear = 0.0;
        }

        double vxEffective = Math.signum(vx) * Math.max(Math.abs(vx), SPEED_EPS);
        double alphaFront = delta - Math.atan2(vy + lf * r, vxEffective);
        double alphaRear = -Math.atan2(vy - lr * r, vxEffective);

        double fyFront0 = vehicle.frontCorneringStiffness() * alphaFront;
        double fyRear0 = vehicle.rearCorneringStiffness() * alphaRear;

        double fxFrontCommand = -BRAKE_FRONT_FRACTION * brakeForce;
        double fxRearCommand = driveForce - (1.0 - BRAKE_FRONT_FRACTION) * brakeForce;
        if (Math.abs(vx) > SPEED_EPS) {
            fxRearCommand -= Math.signum(vx) * rollingResistance;
        }

        double[] frontForce = capTireForce(
                fxFrontCommand, fyFront0, road.frictionCoefficient() * fzFront);
        double[] rearForce = capTireForce(
                fxRearCommand, fyRear0, road.frictionCoefficient() * fzRear);
        double fxFront = frontForce[0];
        double fyFront = frontForce[1];
        double fxRear = rearForce[0];
        double fyRear = rearForce[1];

        double ax = (fxFront + fxRear) / mass;
        double ay = (fyFront + fyRear) / mass;

        checkFinite(ax, ay, vx, vy, r);

        double[] derivatives = new double[6];
        derivatives[VX] = ax + r * vy;
        derivatives[VY] = ay - r * vx;
        derivatives[YAW_RATE] = (lf * fyFront - lr * fyRear) / vehicle.yawInertia();
        derivatives[YAW] = r;
        derivatives[X] = vx * Math.cos(state.yaw()) - vy * Math.sin(state.yaw());
        derivatives[Y] = vx * Math.sin(state.yaw()) + vy * Math.cos(state.yaw());

        return new DynamicsResult(derivatives, ax, ay);
    }

    /**
     * Blends the integrated state toward a kinematic bicycle model at low
     * speed. The blend weight is a smoothstep function of forward speed.
     */
    public static VehicleState applyLowSpeedBlend(
            VehicleState state, double steeringAngle, VehicleParameters vehicle) {
        double vx = state.vx();
        double weight = smoothstep((vx - LOW_SPEED) / (HIGH_SPEED - LOW_SPEED));
        if (weight >= 1.0) {
            return state;
        }
        double wheelbase = vehicle.wheelbase();
        double beta = Math.atan(vehicle.rearAxleDistance() * Math.tan(steeringAngle) / wheelbase);
        double vyKin = vx * Math.tan(beta);
        double yawRateKin = vx * Math.tan(steeringAngle) / wheelbase;
        double vy = weight * state.vy() + (1.0 - weight) * vyKin;
        double yawRate = weight * state.yawRate() + (1.0 - weight) * yawRateKin;
        return new VehicleState(
                state.time(), state.x(), state.y(), vx, vy,
                state.ax(), state.ay(), state.yaw(), yawRate);
    }

    /**
     * Caps the combined longitudinal and lateral tire force to the friction
     * ellipse. Returns a two-element array [fx, fy] with the capped forces.
     */
    private static double[] capTireForce(double fx, double fy, double limit) {
        if (limit <= 0.0) {
            return new double[]{0.0, 0.0};
        }
        double fxAbs = Math.abs(fx);
        if (fxAbs >= limit) {
            return new double[]{Math.signum(fx) * limit, 0.0};
        }
        double fyLimit = Math.sqrt(limit * limit - fx * fx);
        double fyCapped = Math.signum(fy) * Math.min(Math.abs(fy), fyLimit);
        if (fxAbs == 0.0 && fyCapped == 0.0) {
            return new double[]{0.0, 0.0};
        }
        return new double[]{fx, fyCapped};
    }

    private static double smoothstep(double t) {
        double clamped = Math.max(0.0, Math.min(1.0, t));
        return clamped * clamped * (3.0 - 2.0 * clamped);
    }

    private static void checkFinite(double ax, double ay, double vx, double vy, double yawRate) {
        if (!Double.isFinite(ax) || !Double.isFinite(ay)
                || !Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(yawRate)) {
            throw new DivergenceException("Numerical divergence detected: non-finite state or acceleration");
        }
    }
}
