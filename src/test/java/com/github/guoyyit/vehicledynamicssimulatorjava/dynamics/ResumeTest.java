package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ResumeTest {

    private static final double EPS = 1e-10;

    @Test
    void resumeMatchesOneShot() {
        Scenario scenario = TestScenarios.scenario("resume-1",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence oneShot = new Simulator(scenario, config, null).run();

        List<Checkpoint> checkpoints = new ArrayList<>();
        CheckpointCallback callback = (step, time, state) ->
                checkpoints.add(new Checkpoint(scenario.id(), step, time, state));
        new Simulator(scenario, config, callback).run();

        Checkpoint latest = checkpoints.get(checkpoints.size() - 1);
        StateSequence resumed = Simulator.resume(scenario, config, latest, null).run();

        int resumeStart = latest.stepIndex();
        List<VehicleState> oneShotTail = oneShot.stepStates().subList(
                resumeStart, oneShot.stepStates().size());
        List<VehicleState> resumedStates = resumed.stepStates();

        assertEquals(oneShotTail.size(), resumedStates.size(),
                "Resumed step count mismatch: resume started at step " + resumeStart);
        for (int i = 0; i < oneShotTail.size(); i++) {
            VehicleState a = oneShotTail.get(i);
            VehicleState b = resumedStates.get(i);
            assertEquals(a.time(), b.time(), EPS, "time mismatch at tail index " + i);
            assertEquals(a.x(), b.x(), EPS, "x mismatch at tail index " + i);
            assertEquals(a.y(), b.y(), EPS, "y mismatch at tail index " + i);
            assertEquals(a.vx(), b.vx(), EPS, "vx mismatch at tail index " + i);
            assertEquals(a.vy(), b.vy(), EPS, "vy mismatch at tail index " + i);
            assertEquals(a.ax(), b.ax(), EPS, "ax mismatch at tail index " + i);
            assertEquals(a.ay(), b.ay(), EPS, "ay mismatch at tail index " + i);
            assertEquals(a.yaw(), b.yaw(), EPS, "yaw mismatch at tail index " + i);
            assertEquals(a.yawRate(), b.yawRate(), EPS, "yawRate mismatch at tail index " + i);
        }
    }

    @Test
    void resumeFromMiddleCheckpoint() {
        Scenario scenario = TestScenarios.scenario("resume-2",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence oneShot = new Simulator(scenario, config, null).run();

        int checkpointStep = 300;
        VehicleState checkpointState = oneShot.stepStates().get(checkpointStep);
        Checkpoint checkpoint = new Checkpoint(scenario.id(), checkpointStep,
                checkpointState.time(), checkpointState);

        StateSequence resumed = Simulator.resume(scenario, config, checkpoint, null).run();

        List<VehicleState> oneShotTail = oneShot.stepStates().subList(
                checkpointStep, oneShot.stepStates().size());
        assertEquals(oneShotTail.size(), resumed.stepStates().size());
        for (int i = 0; i < oneShotTail.size(); i++) {
            VehicleState a = oneShotTail.get(i);
            VehicleState b = resumed.stepStates().get(i);
            assertEquals(a.time(), b.time(), EPS);
            assertEquals(a.x(), b.x(), EPS);
            assertEquals(a.vx(), b.vx(), EPS);
            assertEquals(a.yaw(), b.yaw(), EPS);
        }
    }

    @Test
    void checkpointTimestampsAreAligned() {
        Scenario scenario = TestScenarios.scenario("resume-3",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        List<Checkpoint> checkpoints = new ArrayList<>();
        CheckpointCallback callback = (step, time, state) ->
                checkpoints.add(new Checkpoint(scenario.id(), step, time, state));
        new Simulator(scenario, config, callback).run();

        for (int i = 0; i < checkpoints.size(); i++) {
            Checkpoint cp = checkpoints.get(i);
            double expectedTime = cp.stepIndex() * config.stepSize();
            assertEquals(expectedTime, cp.time(), EPS,
                    "Checkpoint " + i + " time misaligned: step=" + cp.stepIndex());
            assertEquals(expectedTime, cp.state().time(), EPS);
        }
    }

    @Test
    void multipleResumeCyclesProduceIdenticalResults() {
        Scenario scenario = TestScenarios.scenario("resume-4",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence oneShot = new Simulator(scenario, config, null).run();

        Checkpoint firstCheckpoint = new Checkpoint(scenario.id(), 200,
                oneShot.stepStates().get(200).time(), oneShot.stepStates().get(200));
        StateSequence firstResume = Simulator.resume(scenario, config, firstCheckpoint, null).run();

        Checkpoint secondCheckpoint = new Checkpoint(scenario.id(), 500,
                firstResume.stepStates().get(300).time(), firstResume.stepStates().get(300));
        StateSequence secondResume = Simulator.resume(scenario, config, secondCheckpoint, null).run();

        List<VehicleState> oneShotTail = oneShot.stepStates().subList(
                500, oneShot.stepStates().size());
        assertEquals(oneShotTail.size(), secondResume.stepStates().size());
        for (int i = 0; i < oneShotTail.size(); i++) {
            VehicleState a = oneShotTail.get(i);
            VehicleState b = secondResume.stepStates().get(i);
            assertEquals(a.time(), b.time(), EPS);
            assertEquals(a.x(), b.x(), EPS);
            assertEquals(a.vx(), b.vx(), EPS);
            assertEquals(a.yaw(), b.yaw(), EPS);
        }
    }
}
