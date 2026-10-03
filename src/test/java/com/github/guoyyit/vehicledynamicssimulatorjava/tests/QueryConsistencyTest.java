package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.core.StateSampler;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Requirement 4: configurable step size and query times; sampling at any
 * instant is deterministic and consistent with neighbouring steps.
 */
class QueryConsistencyTest {

    private final SimulationService service = new SimulationService();

    @Test
    void queryOnStepTimesReturnsStepStatesExactly() {
        Scenario scenario = TestScenarios.mixedDriving("q", 0.9);
        List<Double> queryTimes = List.of(0.0, 1.0, 3.33, 8.0);
        SimulationConfig config = new SimulationConfig(0.01, queryTimes, 1);

        SimulationResult result = service.runScenario("q-1", scenario, config);

        for (int i = 0; i < queryTimes.size(); i++) {
            double t = queryTimes.get(i);
            int stepIndex = (int) Math.round(t / 0.01);
            assertEquals(result.states().get(stepIndex), result.queried().get(i),
                    "query at step time " + t + " must return the step state");
        }
    }

    @Test
    void queryBetweenStepsIsInterpolatedConsistently() {
        Scenario scenario = TestScenarios.mixedDriving("qi", 0.9);
        double queryTime = 2.505; // between steps 250 and 251
        SimulationConfig config = new SimulationConfig(0.01, List.of(queryTime), 1);

        SimulationResult result = service.runScenario("qi-1", scenario, config);
        VehicleState sampled = result.queried().get(0);
        VehicleState lo = result.states().get(250);
        VehicleState hi = result.states().get(251);

        assertEquals(queryTime, sampled.time(), 1e-12);
        assertTrue(isBetween(lo.x(), hi.x(), sampled.x()), "x not between neighbours");
        assertTrue(isBetween(lo.longitudinalVx(), hi.longitudinalVx(), sampled.longitudinalVx()),
                "vx not between neighbours");
        assertTrue(isBetween(lo.heading(), hi.heading(), sampled.heading()),
                "heading not between neighbours");
    }

    @Test
    void repeatedSamplingIsDeterministic() {
        Scenario scenario = TestScenarios.mixedDriving("qd", 0.9);
        SimulationResult result = service.runScenario("qd-1", scenario,
                TestScenarios.defaultConfig());
        StateSampler sampler = new StateSampler();
        VehicleState a = sampler.sample(result.states(), 5.4321);
        VehicleState b = sampler.sample(result.states(), 5.4321);
        assertEquals(a, b);
    }

    @Test
    void queryOutsideIntervalIsRejected() {
        Scenario scenario = TestScenarios.mixedDriving("qo", 0.9);
        SimulationResult result = service.runScenario("qo-1", scenario,
                TestScenarios.defaultConfig());
        StateSampler sampler = new StateSampler();
        assertThrows(SimulationException.class, () -> sampler.sample(result.states(), -0.1));
        assertThrows(SimulationException.class, () -> sampler.sample(result.states(), 8.1));
        // And the service validates query times up front.
        assertThrows(SimulationException.class, () -> service.runScenario("qo-2", scenario,
                new SimulationConfig(0.01, List.of(9.0), 1)));
    }

    private static boolean isBetween(double a, double b, double value) {
        double lo = Math.min(a, b) - 1e-12;
        double hi = Math.max(a, b) + 1e-12;
        return value >= lo && value <= hi;
    }
}
