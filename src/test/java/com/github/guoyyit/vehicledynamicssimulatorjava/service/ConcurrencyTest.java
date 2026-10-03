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
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

class ConcurrencyTest {

    private ScenarioService service;

    @BeforeEach
    void setUp(@TempDir Path tempDir) {
        SnapshotStore store = new FileSnapshotStore(tempDir);
        ScenarioRepository repository = new ScenarioRepository(store);
        service = new ScenarioService(repository, store);
    }

    @Test
    void multipleScenariosRunConcurrentlyWithoutInterference() throws Exception {
        int scenarioCount = 6;
        ExecutorService executor = Executors.newFixedThreadPool(3);
        List<Future<StateSequence>> futures = new ArrayList<>();

        for (int i = 0; i < scenarioCount; i++) {
            final int index = i;
            futures.add(executor.submit(() -> {
                Scenario scenario = TestScenarios.scenario("concurrent-" + index,
                        TestScenarios.defaultVehicle(),
                        index % 2 == 0 ? TestScenarios.highFrictionRoad() : TestScenarios.lowFrictionRoad(),
                        TestScenarios.accelerateBrakeSchedule());
                String id = service.createScenario(scenario);
                SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);
                return service.runScenario(id, config);
            }));
        }

        List<StateSequence> sequences = new ArrayList<>();
        for (Future<StateSequence> future : futures) {
            sequences.add(future.get());
        }
        executor.shutdown();

        for (int i = 0; i < sequences.size(); i++) {
            for (int j = i + 1; j < sequences.size(); j++) {
                if (i % 2 == j % 2) {
                    DeterminismTest.assertSequencesEqual(sequences.get(i), sequences.get(j));
                }
            }
        }
    }

    @Test
    void sameScenarioConcurrentRunsAreIdentical() throws Exception {
        Scenario scenario = TestScenarios.scenario("same-concurrent",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        int repetitions = 8;
        ExecutorService executor = Executors.newFixedThreadPool(4);
        List<Future<StateSequence>> futures = new ArrayList<>();
        for (int i = 0; i < repetitions; i++) {
            futures.add(executor.submit(() -> service.runScenario(id, config)));
        }

        List<StateSequence> sequences = new ArrayList<>();
        for (Future<StateSequence> future : futures) {
            sequences.add(future.get());
        }
        executor.shutdown();

        StateSequence reference = sequences.get(0);
        for (int i = 1; i < sequences.size(); i++) {
            DeterminismTest.assertSequencesEqual(reference, sequences.get(i));
        }
    }

    @Test
    void concurrentRunsAndResumesDoNotCorruptState() throws Exception {
        Scenario scenario = TestScenarios.scenario("concurrent-resume",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        ExecutorService executor = Executors.newFixedThreadPool(4);
        List<Future<StateSequence>> futures = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            futures.add(executor.submit(() -> service.runScenario(id, config)));
        }
        for (int i = 0; i < 4; i++) {
            futures.add(executor.submit(() -> service.resumeScenario(id, config)));
        }

        List<StateSequence> sequences = new ArrayList<>();
        for (Future<StateSequence> future : futures) {
            sequences.add(future.get());
        }
        executor.shutdown();

        StateSequence reference = sequences.get(0);
        for (int i = 1; i < 4; i++) {
            DeterminismTest.assertSequencesEqual(reference, sequences.get(i));
        }
    }
}
