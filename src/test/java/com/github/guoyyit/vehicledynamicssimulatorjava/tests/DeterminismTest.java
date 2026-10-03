package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Requirement 1: identical scenario + config must reproduce identical state
 * sequences point by point.
 */
class DeterminismTest {

    private final SimulationService service = new SimulationService();

    @Test
    void repeatedRunsArePointWiseIdentical() {
        Scenario scenario = TestScenarios.mixedDriving("det", 0.9);
        SimulationConfig config = new SimulationConfig(0.01, List.of(0.5, 2.5, 7.9), 1);

        SimulationResult first = service.runScenario("det-1", scenario, config);
        SimulationResult second = service.runScenario("det-2", scenario, config);

        assertEquals(first.states(), second.states());
        assertEquals(first.queried(), second.queried());
    }

    @Test
    void outputContainsAllRequiredChannels() {
        Scenario scenario = TestScenarios.mixedDriving("channels", 0.9);
        SimulationResult result = service.runScenario("channels-1", scenario,
                TestScenarios.defaultConfig());

        List<VehicleState> states = result.states();
        assertEquals(801, states.size()); // t = 0 .. 8.0 at dt = 0.01
        VehicleState late = states.get(700);
        assertEquals(7.0, late.time(), 1e-9);
        // Vehicle has moved, turned and carries full dynamic channels.
        assertTrue(late.x() > 1.0);
        assertTrue(Math.abs(late.heading()) > 1e-3);
        assertTrue(Math.abs(late.yawRate()) >= 0.0); // channel present
        assertTrue(late.speed() > 0.0);
    }

    @Test
    void lowAndHighFrictionBehaveDifferentlyButStayStable() {
        Scenario icy = TestScenarios.aggressiveCornering("icy", 0.2);
        Scenario dry = TestScenarios.aggressiveCornering("dry", 1.0);
        SimulationConfig config = TestScenarios.defaultConfig();

        SimulationResult icyResult = service.runScenario("icy-1", icy, config);
        SimulationResult dryResult = service.runScenario("dry-1", dry, config);

        double icyMaxLateral = maxLateralAccel(icyResult.states());
        double dryMaxLateral = maxLateralAccel(dryResult.states());
        // Low mu caps lateral acceleration well below high mu.
        assertTrue(icyMaxLateral < dryMaxLateral,
                "icy lateral " + icyMaxLateral + " should be below dry " + dryMaxLateral);
        // Both stay finite and bounded (no divergence, no oscillation blow-up).
        assertTrue(maxYawRate(icyResult.states()) < 10.0);
        assertTrue(maxYawRate(dryResult.states()) < 10.0);
    }

    private static double maxLateralAccel(List<VehicleState> states) {
        return states.stream().mapToDouble(s -> Math.abs(s.lateralAy())).max().orElseThrow();
    }

    private static double maxYawRate(List<VehicleState> states) {
        return states.stream().mapToDouble(s -> Math.abs(s.yawRate())).max().orElseThrow();
    }
}
