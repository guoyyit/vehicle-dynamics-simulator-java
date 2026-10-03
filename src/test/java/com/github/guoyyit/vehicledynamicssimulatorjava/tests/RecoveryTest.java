package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.ScenarioStatus;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Requirement 5: resuming from the latest checkpoint reproduces the
 * one-shot sequence point by point, without time-axis shifts or state jumps.
 */
class RecoveryTest {

    private final SimulationService service = new SimulationService();

    @Test
    void resumedRunMatchesOneShotRunPointWise() {
        Scenario scenario = TestScenarios.mixedDriving("rec", 0.9);
        SimulationConfig config = new SimulationConfig(0.01, List.of(), 1);
        double stopTime = 3.5;

        SimulationResult oneShot = service.runScenario("rec-full", scenario, config);

        // Simulate a crash at t=3.5: the chunked run stops, checkpoints stay.
        service.runScenarioUntil("rec-chunk", scenario, config, stopTime);
        assertEquals(ScenarioStatus.RUNNING, service.statusOf("rec-chunk"));

        SimulationResult resumed = service.resumeScenario("rec-chunk", scenario, config);
        assertEquals(ScenarioStatus.COMPLETED, service.statusOf("rec-chunk"));

        int stopIndex = (int) Math.round(stopTime / 0.01);
        List<VehicleState> tail = oneShot.states().subList(stopIndex, oneShot.states().size());
        assertEquals(tail, resumed.states());
        // No time-axis shift: continuation starts exactly on the grid point.
        assertEquals(stopTime, resumed.states().get(0).time(), 1e-9);
    }

    @Test
    void resumeUsesLatestCheckpointOnly() {
        Scenario scenario = TestScenarios.mixedDriving("rec2", 0.9);
        // Checkpoints every 50 steps; crash at t=3.33 -> latest checkpoint at t=3.0.
        SimulationConfig config = new SimulationConfig(0.01, List.of(), 50);

        SimulationResult oneShot = service.runScenario("rec2-full", scenario, config);
        service.runScenarioUntil("rec2-chunk", scenario, config, 3.33);
        SimulationResult resumed = service.resumeScenario("rec2-chunk", scenario, config);

        int checkpointIndex = 300;
        List<VehicleState> tail = oneShot.states().subList(checkpointIndex, oneShot.states().size());
        assertEquals(tail, resumed.states());
    }

    @Test
    void resumeWithoutCheckpointRunsFromStart() {
        Scenario scenario = TestScenarios.mixedDriving("rec3", 0.9);
        SimulationConfig config = TestScenarios.defaultConfig();

        SimulationResult oneShot = service.runScenario("rec3-full", scenario, config);
        SimulationResult resumed = service.resumeScenario("rec3-fresh", scenario, config);

        assertEquals(oneShot.states(), resumed.states());
    }

    @Test
    void chunkedStitchingEqualsOneShot() {
        Scenario scenario = TestScenarios.mixedDriving("rec4", 0.9);
        SimulationConfig config = TestScenarios.defaultConfig();

        SimulationResult oneShot = service.runScenario("rec4-full", scenario, config);
        SimulationResult firstChunk = service.runScenarioUntil("rec4-chunk", scenario, config, 2.0);
        SimulationResult secondChunk = service.resumeScenario("rec4-chunk", scenario, config);

        List<VehicleState> stitched = new java.util.ArrayList<>(firstChunk.states());
        stitched.addAll(secondChunk.states().subList(1, secondChunk.states().size()));
        assertEquals(oneShot.states(), stitched);
    }
}
