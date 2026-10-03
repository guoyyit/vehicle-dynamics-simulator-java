package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DriverInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingSchedule;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

class FrictionLimitTest {

    private static final double GRAVITY = 9.81;
    private static final double TOLERANCE = 1.01;

    @Test
    void accelerationNeverExceedsFrictionLimit() {
        Scenario scenario = new Scenario("friction-1",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(scenario, config, null).run();
        double limit = 0.9 * GRAVITY;
        for (VehicleState state : sequence.stepStates()) {
            double accel = Math.hypot(state.ax(), state.ay());
            assertTrue(accel <= limit * TOLERANCE,
                    "Acceleration " + accel + " exceeds limit " + limit + " at t=" + state.time());
        }
    }

    @Test
    void lowFrictionProducesLowerAccelThanHighFriction() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        DrivingSchedule schedule = TestScenarios.accelerateBrakeSchedule();
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence highMu = new Simulator(
                new Scenario("high-mu", vehicle, TestScenarios.highFrictionRoad(), schedule),
                config, null).run();
        StateSequence lowMu = new Simulator(
                new Scenario("low-mu", vehicle, TestScenarios.lowFrictionRoad(), schedule),
                config, null).run();

        double highMuMax = maxAccel(highMu);
        double lowMuMax = maxAccel(lowMu);

        assertTrue(lowMuMax < highMuMax,
                "Low-friction max accel " + lowMuMax + " should be below high-friction " + highMuMax);
        assertTrue(lowMuMax <= 0.3 * GRAVITY * TOLERANCE,
                "Low-friction accel " + lowMuMax + " exceeds 0.3g limit");
    }

    @Test
    void noOscillationNearLimit() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        RoadCondition road = TestScenarios.highFrictionRoad();
        DrivingSchedule schedule = new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(2.0, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(2.01, new DriverInput(0.0, 1.0, 0.0)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.0, 1.0, 0.0))));
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(
                new Scenario("oscillation", vehicle, road, schedule), config, null).run();

        int signChanges = 0;
        double previousAx = 0.0;
        for (VehicleState state : sequence.stepStates()) {
            if (Math.abs(state.ax()) > 0.5) {
                if (previousAx != 0.0 && Math.signum(state.ax()) != Math.signum(previousAx)) {
                    signChanges++;
                }
                previousAx = state.ax();
            }
        }
        assertTrue(signChanges <= 2,
                "Acceleration oscillates near limit: " + signChanges + " sign changes");
    }

    @Test
    void combinedSlipRespectsFrictionCircle() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        RoadCondition road = TestScenarios.highFrictionRoad();
        DrivingSchedule schedule = TestScenarios.steeringSchedule();
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(
                new Scenario("combined", vehicle, road, schedule), config, null).run();

        double limit = 0.9 * GRAVITY;
        for (VehicleState state : sequence.stepStates()) {
            double accel = Math.hypot(state.ax(), state.ay());
            assertTrue(accel <= limit * TOLERANCE,
                    "Combined acceleration " + accel + " exceeds limit at t=" + state.time());
        }
    }

    private static double maxAccel(StateSequence sequence) {
        double max = 0.0;
        for (VehicleState state : sequence.stepStates()) {
            double accel = Math.hypot(state.ax(), state.ay());
            if (accel > max) {
                max = accel;
            }
        }
        return max;
    }
}
