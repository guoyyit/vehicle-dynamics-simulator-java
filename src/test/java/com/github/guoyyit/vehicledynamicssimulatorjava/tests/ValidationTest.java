package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.checkpoint.CheckpointStore;
import com.github.guoyyit.vehicledynamicssimulatorjava.core.SimulationEngine;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.ScenarioStatus;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Requirement 7: illegal input, out-of-range parameters and numerical
 * divergence raise explicit errors and fail the scenario; no partial or
 * silently wrong output is produced.
 */
class ValidationTest {

    private final SimulationService service = new SimulationService();

    @Test
    void rejectsInvalidVehicleParameters() {
        VehicleParameters bad = new VehicleParameters(
                -1500.0, 2500.0, 1.2, 1.4, 60000.0, 65000.0, 0.6, 3.0, 6.0);
        Scenario scenario = new Scenario("bad-mass", bad, 0.9,
                List.of(new DrivingInput(0.0, 1.0, 0.0)), 5.0);
        SimulationException e = assertThrows(SimulationException.class,
                () -> service.runScenario("v-1", scenario, TestScenarios.defaultConfig()));
        assertTrue(e.getMessage().contains("mass"));
        assertEquals(ScenarioStatus.FAILED, service.statusOf("v-1"));
    }

    @Test
    void rejectsNonPositiveFrictionAndDuration() {
        Scenario icy = TestScenarios.mixedDriving("mu0", 0.0);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-2", icy, TestScenarios.defaultConfig()));

        Scenario zeroDuration = new Scenario("dur0", TestScenarios.passengerCar(), 0.9,
                List.of(new DrivingInput(0.0, 1.0, 0.0)), 0.0);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-3", zeroDuration, TestScenarios.defaultConfig()));
    }

    @Test
    void rejectsInvalidConfig() {
        Scenario scenario = TestScenarios.mixedDriving("cfg", 0.9);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-4", scenario, new SimulationConfig(0.0, List.of(), 1)));
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-5", scenario, new SimulationConfig(0.01, List.of(), 0)));
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-6", scenario,
                        new SimulationConfig(0.01, List.of(99.0), 1)));
    }

    @Test
    void rejectsMalformedInputSchedule() {
        List<DrivingInput> unsorted = List.of(
                new DrivingInput(2.0, 1.0, 0.0),
                new DrivingInput(1.0, 0.0, 0.0));
        Scenario scenario = new Scenario("unsorted", TestScenarios.passengerCar(), 0.9,
                unsorted, 5.0);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-7", scenario, TestScenarios.defaultConfig()));

        List<DrivingInput> outOfRange = List.of(new DrivingInput(99.0, 1.0, 0.0));
        Scenario scenario2 = new Scenario("oor", TestScenarios.passengerCar(), 0.9,
                outOfRange, 5.0);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-8", scenario2, TestScenarios.defaultConfig()));

        List<DrivingInput> nan = List.of(new DrivingInput(0.0, Double.NaN, 0.0));
        Scenario scenario3 = new Scenario("nan", TestScenarios.passengerCar(), 0.9, nan, 5.0);
        assertThrows(SimulationException.class,
                () -> service.runScenario("v-9", scenario3, TestScenarios.defaultConfig()));
    }

    @Test
    void numericalDivergenceRaisesExplicitError() {
        // Bypass service validation to exercise the engine's divergence guard:
        // a NaN parameter must surface as a SimulationException, not silently
        // propagate into the output.
        VehicleParameters nanParams = new VehicleParameters(
                Double.NaN, 2500.0, 1.2, 1.4, 60000.0, 65000.0, 0.6, 3.0, 6.0);
        Scenario scenario = new Scenario("diverge", nanParams, 0.9,
                List.of(new DrivingInput(0.0, 1.0, 0.0)), 5.0);
        SimulationEngine engine = new SimulationEngine();
        assertThrows(SimulationException.class,
                () -> engine.run("div-1", scenario, TestScenarios.defaultConfig(),
                        new CheckpointStore()));
    }
}
