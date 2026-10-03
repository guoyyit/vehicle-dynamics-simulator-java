package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests that identical scenarios and configurations produce identical state
 * sequences, both sequentially and concurrently.
 */
public class DeterminismTest {

    private static final double EPS = 1e-12;

    @Test
    void repeatedRunsAreIdentical() {
        Scenario scenario = TestScenarios.scenario("det-1",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 2.5, 5.0, 7.5, 10.0);

        StateSequence first = new Simulator(scenario, config, null).run();
        StateSequence second = new Simulator(scenario, config, null).run();

        assertSequencesEqual(first, second);
    }

    @Test
    void concurrentRepeatedRunsAreIdentical() throws Exception {
        Scenario scenario = TestScenarios.scenario("det-2",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        int repetitions = 8;
        ExecutorService executor = Executors.newFixedThreadPool(4);
        List<Future<StateSequence>> futures = new ArrayList<>();
        for (int i = 0; i < repetitions; i++) {
            futures.add(executor.submit(() -> new Simulator(scenario, config, null).run()));
        }
        List<StateSequence> sequences = new ArrayList<>();
        for (Future<StateSequence> future : futures) {
            sequences.add(future.get());
        }
        executor.shutdown();

        StateSequence reference = sequences.get(0);
        for (int i = 1; i < sequences.size(); i++) {
            assertSequencesEqual(reference, sequences.get(i));
        }
    }

    @Test
    void queryStatesAreDeterministic() {
        Scenario scenario = TestScenarios.scenario("det-3",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.3, 1.7, 3.3, 5.8, 9.1);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        for (int i = 0; i < 5; i++) {
            StateSequence rerun = new Simulator(scenario, config, null).run();
            for (int j = 0; j < sequence.queryStates().size(); j++) {
                assertStateEquals(sequence.queryStates().get(j), rerun.queryStates().get(j));
            }
        }
    }

    public static void assertSequencesEqual(StateSequence a, StateSequence b) {
        assertEquals(a.stepStates().size(), b.stepStates().size(), "Step count mismatch");
        for (int i = 0; i < a.stepStates().size(); i++) {
            assertStateEquals(a.stepStates().get(i), b.stepStates().get(i));
        }
        assertEquals(a.queryStates().size(), b.queryStates().size(), "Query count mismatch");
        for (int i = 0; i < a.queryStates().size(); i++) {
            assertStateEquals(a.queryStates().get(i), b.queryStates().get(i));
        }
    }

    public static void assertStateEquals(VehicleState a, VehicleState b) {
        assertEquals(a.time(), b.time(), EPS, "time mismatch");
        assertEquals(a.x(), b.x(), EPS, "x mismatch");
        assertEquals(a.y(), b.y(), EPS, "y mismatch");
        assertEquals(a.vx(), b.vx(), EPS, "vx mismatch");
        assertEquals(a.vy(), b.vy(), EPS, "vy mismatch");
        assertEquals(a.ax(), b.ax(), EPS, "ax mismatch");
        assertEquals(a.ay(), b.ay(), EPS, "ay mismatch");
        assertEquals(a.yaw(), b.yaw(), EPS, "yaw mismatch");
        assertEquals(a.yawRate(), b.yawRate(), EPS, "yawRate mismatch");
    }
}
