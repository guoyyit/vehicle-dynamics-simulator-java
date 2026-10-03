package com.github.guoyyit.vehicledynamicssimulatorjava.tests;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;

import java.util.List;

/**
 * Shared fixtures: a typical passenger car and a mixed drive/brake/steer
 * scenario used across the test suite.
 */
final class TestScenarios {

    private TestScenarios() {
    }

    static VehicleParameters passengerCar() {
        return new VehicleParameters(
                1500.0,   // mass [kg]
                2500.0,   // yaw inertia [kg*m^2]
                1.2,      // lf [m]
                1.4,      // lr [m]
                60000.0,  // Cf [N/rad]
                65000.0,  // Cr [N/rad]
                0.6,      // max steer [rad]
                3.0,      // max drive accel [m/s^2]
                6.0);     // max brake accel [m/s^2]
    }

    static Scenario mixedDriving(String id, double mu) {
        List<DrivingInput> inputs = List.of(
                new DrivingInput(0.0, 2.5, 0.0),
                new DrivingInput(2.0, 0.5, 0.06),
                new DrivingInput(4.0, -2.0, 0.10),
                new DrivingInput(6.0, 0.0, -0.05));
        return new Scenario(id, passengerCar(), mu, inputs, 8.0);
    }

    static Scenario aggressiveCornering(String id, double mu) {
        List<DrivingInput> inputs = List.of(
                new DrivingInput(0.0, 3.0, 0.0),
                new DrivingInput(3.0, 0.0, 0.35),
                new DrivingInput(6.0, -4.0, 0.35));
        return new Scenario(id, passengerCar(), mu, inputs, 9.0);
    }

    static SimulationConfig defaultConfig() {
        return new SimulationConfig(0.01, List.of(), 1);
    }
}
