package com.github.guoyyit.vehicledynamicssimulatorjava;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.DriverInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingSchedule;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;

import java.util.List;

/**
 * Factory methods for test scenarios and configurations.
 */
public final class TestScenarios {

    private TestScenarios() {
    }

    public static VehicleParameters defaultVehicle() {
        return new VehicleParameters(
                1500.0, 2.7, 1.2, 1.5, 0.5, 2500.0,
                80000.0, 80000.0, 0.6, 4000.0, 8000.0);
    }

    public static RoadCondition highFrictionRoad() {
        return new RoadCondition(0.9, 0.015);
    }

    public static RoadCondition lowFrictionRoad() {
        return new RoadCondition(0.3, 0.015);
    }

    /**
     * Schedule: accelerate, then brake to a stop.
     */
    public static DrivingSchedule accelerateBrakeSchedule() {
        return new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(4.99, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(5.0, new DriverInput(0.0, 0.8, 0.0)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.0, 0.8, 0.0))));
    }

    /**
     * Schedule: constant speed with a steering input.
     */
    public static DrivingSchedule steeringSchedule() {
        return new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.6, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(1.0, new DriverInput(0.6, 0.0, 0.1)),
                new DrivingSchedule.TimedInput(5.0, new DriverInput(0.6, 0.0, 0.1)),
                new DrivingSchedule.TimedInput(6.0, new DriverInput(0.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.0, 0.0, 0.0))));
    }

    public static Scenario scenario(String id, VehicleParameters vehicle, RoadCondition road,
                                    DrivingSchedule schedule) {
        return new Scenario(id, vehicle, road, schedule);
    }

    public static SimulationConfig config(double dt, double... queryTimes) {
        return new SimulationConfig(dt, queryTimes, 100);
    }

    public static SimulationConfig config(double dt, int checkpointInterval, double... queryTimes) {
        return new SimulationConfig(dt, queryTimes, checkpointInterval);
    }
}
