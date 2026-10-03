package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.core.VehicleDynamicsModel;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Requirement 3: combined acceleration never exceeds mu * g; approaching the
 * limit must not diverge or oscillate persistently.
 */
class FrictionLimitTest {

    private final SimulationService service = new SimulationService();

    @Test
    void combinedAccelerationNeverExceedsFrictionLimit() {
        for (double mu : new double[]{0.2, 0.5, 0.9, 1.2}) {
            Scenario scenario = TestScenarios.aggressiveCornering("mu-" + mu, mu);
            SimulationResult result = service.runScenario("fl-" + mu, scenario,
                    TestScenarios.defaultConfig());
            double limit = mu * VehicleDynamicsModel.GRAVITY;
            for (VehicleState state : result.states()) {
                assertTrue(state.combinedAcceleration() <= limit + 1e-9,
                        "mu=" + mu + " t=" + state.time() + " combined="
                                + state.combinedAcceleration() + " limit=" + limit);
            }
        }
    }

    @Test
    void saturatedTiresStayBoundedWithoutOscillation() {
        // Demand far beyond the limit: full throttle plus full steering on ice.
        Scenario scenario = TestScenarios.aggressiveCornering("saturated", 0.15);
        SimulationResult result = service.runScenario("sat-1", scenario,
                TestScenarios.defaultConfig());
        List<VehicleState> states = result.states();

        double limit = 0.15 * VehicleDynamicsModel.GRAVITY;
        VehicleState last = states.get(states.size() - 1);
        assertTrue(Double.isFinite(last.speed()) && last.speed() < 200.0);
        assertTrue(Math.abs(last.yawRate()) < 5.0);

        // Once saturated, the combined acceleration hugs the limit and does
        // not flap: count sign changes of lateral acceleration in the tail.
        int signChanges = 0;
        double previousSign = 0.0;
        for (int i = states.size() / 2; i < states.size(); i++) {
            double ay = states.get(i).lateralAy();
            double sign = Math.signum(ay);
            if (sign != 0.0 && previousSign != 0.0 && sign != previousSign) {
                signChanges++;
            }
            if (sign != 0.0) {
                previousSign = sign;
            }
        }
        assertTrue(signChanges <= 2, "persistent oscillation detected: " + signChanges);
        // And the saturated phase actually reaches the limit (constraint active).
        double maxCombined = states.stream()
                .mapToDouble(VehicleState::combinedAcceleration).max().orElseThrow();
        assertTrue(maxCombined > 0.8 * limit,
                "expected saturation near the limit, got " + maxCombined);
    }

    @Test
    void clampScalesIntoFrictionCircle() {
        VehicleDynamicsModel model = new VehicleDynamicsModel(TestScenarios.passengerCar(), 0.8);
        double limit = 0.8 * VehicleDynamicsModel.GRAVITY;

        double[] inside = model.clampToFrictionLimit(1.0, 1.0);
        assertEquals(1.0, inside[0], 1e-12);
        assertEquals(1.0, inside[1], 1e-12);

        double[] clamped = model.clampToFrictionLimit(3.0 * limit, 4.0 * limit);
        assertEquals(limit, Math.hypot(clamped[0], clamped[1]), 1e-9);
        // Direction preserved by proportional scaling.
        assertEquals(3.0 / 4.0, clamped[0] / clamped[1], 1e-9);
    }
}
