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

class BehaviorDifferenceTest {

    @Test
    void lowSpeedSteeringProducesKinematicResponse() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        RoadCondition road = TestScenarios.highFrictionRoad();
        DrivingSchedule schedule = new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.15, 0.0, 0.2)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.15, 0.0, 0.2))));
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(
                new Scenario("low-speed", vehicle, road, schedule), config, null).run();

        VehicleState last = sequence.stepStates().get(sequence.stepStates().size() - 1);
        double speed = Math.hypot(last.vx(), last.vy());
        double expectedYawRate = speed * Math.tan(0.2) / vehicle.wheelbase();
        double yawRateError = Math.abs(last.yawRate() - expectedYawRate);
        assertTrue(yawRateError < 0.1 * Math.max(0.01, Math.abs(expectedYawRate)),
                "Low-speed yaw rate " + last.yawRate() + " should match kinematic " + expectedYawRate);
    }

    @Test
    void highSpeedSteeringProducesDynamicResponse() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        RoadCondition road = TestScenarios.highFrictionRoad();
        DrivingSchedule schedule = new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.8, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(3.0, new DriverInput(0.8, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(3.01, new DriverInput(0.8, 0.0, 0.15)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.8, 0.0, 0.15))));
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(
                new Scenario("high-speed", vehicle, road, schedule), config, null).run();

        double maxYawRate = 0.0;
        for (VehicleState state : sequence.stepStates()) {
            if (Math.abs(state.yawRate()) > maxYawRate) {
                maxYawRate = Math.abs(state.yawRate());
            }
        }
        assertTrue(maxYawRate > 0.01, "High-speed steering should produce yaw rate");
        for (VehicleState state : sequence.stepStates()) {
            double accel = Math.hypot(state.ax(), state.ay());
            assertTrue(accel <= 0.9 * 9.81 * 1.01,
                    "Acceleration exceeds limit at t=" + state.time());
        }
    }

    @Test
    void lowFrictionReducesCorneringCapability() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        DrivingSchedule schedule = new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.5, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(2.0, new DriverInput(0.5, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(2.01, new DriverInput(0.5, 0.0, 0.2)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.5, 0.0, 0.2))));
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence highMu = new Simulator(
                new Scenario("high-mu-corner", vehicle, TestScenarios.highFrictionRoad(), schedule),
                config, null).run();
        StateSequence lowMu = new Simulator(
                new Scenario("low-mu-corner", vehicle, TestScenarios.lowFrictionRoad(), schedule),
                config, null).run();

        double highMuMaxLatAccel = maxAbsLateralAccel(highMu);
        double lowMuMaxLatAccel = maxAbsLateralAccel(lowMu);
        assertTrue(lowMuMaxLatAccel < highMuMaxLatAccel,
                "Low-mu lateral accel " + lowMuMaxLatAccel + " should be below high-mu "
                        + highMuMaxLatAccel);
        assertTrue(lowMuMaxLatAccel <= 0.3 * 9.81 * 1.01,
                "Low-mu lateral accel " + lowMuMaxLatAccel + " exceeds 0.3g limit");
    }

    @Test
    void vehicleComesToRestWithoutReversing() {
        VehicleParameters vehicle = TestScenarios.defaultVehicle();
        RoadCondition road = TestScenarios.highFrictionRoad();
        DrivingSchedule schedule = TestScenarios.accelerateBrakeSchedule();
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);

        StateSequence sequence = new Simulator(
                new Scenario("stop", vehicle, road, schedule), config, null).run();

        double minVx = Double.MAX_VALUE;
        for (VehicleState state : sequence.stepStates()) {
            if (state.vx() < minVx) {
                minVx = state.vx();
            }
        }
        assertTrue(minVx >= -0.01, "Vehicle should not reverse, min vx=" + minVx);
        VehicleState last = sequence.stepStates().get(sequence.stepStates().size() - 1);
        assertTrue(Math.abs(last.vx()) < 0.1, "Vehicle should be stopped at end, vx=" + last.vx());
    }

    private static double maxAbsLateralAccel(StateSequence sequence) {
        double max = 0.0;
        for (VehicleState state : sequence.stepStates()) {
            double abs = Math.abs(state.ay());
            if (abs > max) {
                max = abs;
            }
        }
        return max;
    }
}
