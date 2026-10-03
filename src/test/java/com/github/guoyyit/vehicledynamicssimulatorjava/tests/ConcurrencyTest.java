package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Requirement 6: concurrent runs are isolated; concurrent repetition of the
 * same scenario matches sequential execution exactly.
 */
class ConcurrencyTest {

    private final SimulationService service = new SimulationService();

    @Test
    void concurrentRepeatsMatchSequentialRun() throws Exception {
        Scenario scenario = TestScenarios.mixedDriving("conc", 0.9);
        SimulationConfig config = TestScenarios.defaultConfig();
        SimulationResult sequential = service.runScenario("conc-seq", scenario, config);

        int parallelism = 8;
        ExecutorService pool = Executors.newFixedThreadPool(parallelism);
        try {
            List<Callable<SimulationResult>> tasks = new ArrayList<>();
            for (int i = 0; i < parallelism; i++) {
                String runId = "conc-" + i;
                tasks.add(() -> service.runScenario(runId, scenario, config));
            }
            for (Future<SimulationResult> future : pool.invokeAll(tasks)) {
                SimulationResult result = future.get();
                assertEquals(sequential.states(), result.states());
            }
        } finally {
            pool.shutdownNow();
        }
    }

    @Test
    void differentScenariosRunConcurrentlyWithoutInterference() throws Exception {
        List<Scenario> scenarios = List.of(
                TestScenarios.mixedDriving("s-dry", 1.0),
                TestScenarios.aggressiveCornering("s-icy", 0.2),
                TestScenarios.mixedDriving("s-wet", 0.5));
        SimulationConfig config = TestScenarios.defaultConfig();

        List<SimulationResult> expected = new ArrayList<>();
        for (int i = 0; i < scenarios.size(); i++) {
            expected.add(service.runScenario("seq-" + i, scenarios.get(i), config));
        }

        ExecutorService pool = Executors.newFixedThreadPool(scenarios.size());
        try {
            List<Callable<SimulationResult>> tasks = new ArrayList<>();
            for (int i = 0; i < scenarios.size(); i++) {
                String runId = "par-" + i;
                Scenario scenario = scenarios.get(i);
                tasks.add(() -> service.runScenario(runId, scenario, config));
            }
            List<Future<SimulationResult>> futures = pool.invokeAll(tasks);
            for (int i = 0; i < futures.size(); i++) {
                assertEquals(expected.get(i).states(), futures.get(i).get().states(),
                        "scenario " + scenarios.get(i).scenarioId() + " diverged under concurrency");
            }
        } finally {
            pool.shutdownNow();
        }
    }
}
