package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.DeterminismTest;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class SnapshotResumeTest {

    @TempDir
    Path tempDir;

    private ScenarioService service;
    private SnapshotStore store;

    @BeforeEach
    void setUp() {
        store = new FileSnapshotStore(tempDir);
        ScenarioRepository repository = new ScenarioRepository(store);
        service = new ScenarioService(repository, store);
    }

    @Test
    void resumeFromPersistedCheckpointMatchesOneShot() {
        Scenario scenario = TestScenarios.scenario("snapshot-resume",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence oneShot = service.runScenario(id, config);
        StateSequence resumed = service.resumeScenario(id, config);

        int checkpointStep = store.findLatest(id).orElseThrow().stepIndex();
        java.util.List<com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState> oneShotTail =
                oneShot.stepStates().subList(checkpointStep, oneShot.stepStates().size());

        assertEquals(oneShotTail.size(), resumed.stepStates().size(),
                "Resumed step count mismatch");
        for (int i = 0; i < oneShotTail.size(); i++) {
            DeterminismTest.assertStateEquals(oneShotTail.get(i), resumed.stepStates().get(i));
        }
    }

    @Test
    void scenarioAndCheckpointsPersistedAcrossServiceRestart() {
        Scenario scenario = TestScenarios.scenario("restart",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);
        service.runScenario(id, config);

        SnapshotStore restoredStore = new FileSnapshotStore(tempDir);
        ScenarioRepository restoredRepository = new ScenarioRepository(restoredStore);
        ScenarioService restoredService = new ScenarioService(restoredRepository, restoredStore);

        assertNotNull(restoredRepository.findById(id).orElseThrow());
        StateSequence resumed = restoredService.resumeScenario(id, config);
        assertNotNull(resumed);
        assertEquals(ScenarioRepository.Status.COMPLETED, restoredService.statusOf(id));
    }

    @Test
    void multipleCheckpointsAreOrderedAndLatestIsUsed() {
        Scenario scenario = TestScenarios.scenario("multi-checkpoint",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 100, 0.0, 5.0, 10.0);
        service.runScenario(id, config);

        java.util.List<com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.Checkpoint> checkpoints =
                store.list(id);
        for (int i = 1; i < checkpoints.size(); i++) {
            org.junit.jupiter.api.Assertions.assertTrue(
                    checkpoints.get(i - 1).stepIndex() < checkpoints.get(i).stepIndex(),
                    "Checkpoints must be ordered by step index");
        }

        StateSequence resumed = service.resumeScenario(id, config);
        assertNotNull(resumed);
    }
}
