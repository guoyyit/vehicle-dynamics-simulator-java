package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class QueryConsistencyTest {

    private static final double EPS = 1e-9;

    @Test
    void queryAtStepBoundaryReturnsExactStepState() {
        Scenario scenario = TestScenarios.scenario("query-1",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();

        for (int i = 0; i < sequence.stepStates().size(); i += 100) {
            VehicleState step = sequence.stepStates().get(i);
            VehicleState queried = sequence.stateAt(step.time());
            assertEquals(step.time(), queried.time(), EPS);
            assertEquals(step.x(), queried.x(), EPS);
            assertEquals(step.y(), queried.y(), EPS);
            assertEquals(step.vx(), queried.vx(), EPS);
            assertEquals(step.vy(), queried.vy(), EPS);
            assertEquals(step.yaw(), queried.yaw(), EPS);
            assertEquals(step.yawRate(), queried.yawRate(), EPS);
        }
    }

    @Test
    void queryAtArbitraryTimeMatchesInterpolation() {
        Scenario scenario = TestScenarios.scenario("query-2",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();

        double[] testTimes = {0.005, 0.015, 2.345, 4.999, 5.001, 7.777, 9.999};
        for (double time : testTimes) {
            VehicleState queried = sequence.stateAt(time);
            VehicleState expected = interpolateFromSteps(sequence, time);
            assertNotNull(queried);
            assertEquals(expected.x(), queried.x(), 1e-6, "x mismatch at t=" + time);
            assertEquals(expected.y(), queried.y(), 1e-6, "y mismatch at t=" + time);
            assertEquals(expected.vx(), queried.vx(), 1e-6, "vx mismatch at t=" + time);
            assertEquals(expected.vy(), queried.vy(), 1e-6, "vy mismatch at t=" + time);
            assertEquals(expected.yaw(), queried.yaw(), 1e-6, "yaw mismatch at t=" + time);
            assertEquals(expected.yawRate(), queried.yawRate(), 1e-6, "yawRate mismatch at t=" + time);
        }
    }

    @Test
    void queryAtZeroReturnsInitialState() {
        Scenario scenario = TestScenarios.scenario("query-3",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        VehicleState initial = sequence.stepStates().get(0);
        VehicleState queried = sequence.stateAt(0.0);

        assertEquals(initial.time(), queried.time(), EPS);
        assertEquals(initial.x(), queried.x(), EPS);
        assertEquals(initial.y(), queried.y(), EPS);
        assertEquals(initial.vx(), queried.vx(), EPS);
        assertEquals(initial.vy(), queried.vy(), EPS);
        assertEquals(initial.yaw(), queried.yaw(), EPS);
    }

    @Test
    void queryAtHorizonReturnsFinalState() {
        Scenario scenario = TestScenarios.scenario("query-4",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        VehicleState last = sequence.stepStates().get(sequence.stepStates().size() - 1);
        VehicleState queried = sequence.stateAt(10.0);

        assertEquals(last.time(), queried.time(), EPS);
        assertEquals(last.x(), queried.x(), EPS);
        assertEquals(last.vx(), queried.vx(), EPS);
    }

    @Test
    void queryOutsideHorizonThrows() {
        Scenario scenario = TestScenarios.scenario("query-5",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        assertThrows(Exception.class, () -> sequence.stateAt(11.0));
        assertThrows(Exception.class, () -> sequence.stateAt(-1.0));
    }

    @Test
    void queryStatesAreSortedAndConsistent() {
        Scenario scenario = TestScenarios.scenario("query-6",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 9.0, 1.0, 5.0, 3.0, 7.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        double[] queryTimes = config.queryTimes();
        for (int i = 0; i < queryTimes.length; i++) {
            assertEquals(queryTimes[i], sequence.queryStates().get(i).time(), EPS);
        }
        for (int i = 1; i < sequence.queryStates().size(); i++) {
            assertTrue(sequence.queryStates().get(i - 1).time() < sequence.queryStates().get(i).time());
        }
    }

    private VehicleState interpolateFromSteps(StateSequence sequence, double time) {
        for (int i = 0; i < sequence.stepStates().size() - 1; i++) {
            VehicleState a = sequence.stepStates().get(i);
            VehicleState b = sequence.stepStates().get(i + 1);
            if (time >= a.time() - EPS && time <= b.time() + EPS) {
                double fraction = (time - a.time()) / (b.time() - a.time());
                return VehicleState.interpolate(a, b, fraction);
            }
        }
        throw new IllegalArgumentException("Time " + time + " not in step range");
    }
}
