package com.github.guoyyit.vehicledynamicssimulatorjava.core;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

import java.util.List;

/**
 * Deterministic sampling of states at arbitrary query times by linear
 * interpolation between adjacent integration steps.
 *
 * <p>A query that lands exactly on a step time returns that step's state
 * unchanged; any other query is interpolated component-wise between the two
 * bracketing steps, so results stay consistent with the simulated time axis.
 */
public class StateSampler {

    /**
     * Samples the state at {@code time}. The time must lie within the
     * covered interval; results are consistent with neighbouring steps.
     */
    public VehicleState sample(List<VehicleState> states, double time) {
        if (states.isEmpty()) {
            throw new SimulationException("cannot sample an empty state sequence");
        }
        if (time < states.get(0).time() || time > states.get(states.size() - 1).time()) {
            throw new SimulationException("query time " + time + " outside simulated interval ["
                    + states.get(0).time() + ", " + states.get(states.size() - 1).time() + "]");
        }
        int upper = upperBracket(states, time);
        VehicleState hi = states.get(upper);
        if (hi.time() == time || upper == 0) {
            return hi;
        }
        VehicleState lo = states.get(upper - 1);
        double ratio = (time - lo.time()) / (hi.time() - lo.time());
        return interpolate(lo, hi, ratio, time);
    }

    /**
     * Binary search for the smallest index whose state time is >= query time.
     */
    private static int upperBracket(List<VehicleState> states, double time) {
        int lo = 0;
        int hi = states.size() - 1;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (states.get(mid).time() < time) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    private static VehicleState interpolate(VehicleState a, VehicleState b, double ratio, double time) {
        return new VehicleState(
                time,
                lerp(a.x(), b.x(), ratio),
                lerp(a.y(), b.y(), ratio),
                lerp(a.heading(), b.heading(), ratio),
                lerp(a.longitudinalVx(), b.longitudinalVx(), ratio),
                lerp(a.lateralVy(), b.lateralVy(), ratio),
                lerp(a.yawRate(), b.yawRate(), ratio),
                lerp(a.longitudinalAx(), b.longitudinalAx(), ratio),
                lerp(a.lateralAy(), b.lateralAy(), ratio));
    }

    private static double lerp(double a, double b, double ratio) {
        return a + (b - a) * ratio;
    }
}
