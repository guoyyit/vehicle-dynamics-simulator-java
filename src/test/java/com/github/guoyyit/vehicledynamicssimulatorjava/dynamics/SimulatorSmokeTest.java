package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

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

class SimulatorSmokeTest {

    @Test
    void smokeRun() {
        VehicleParameters vehicle = new VehicleParameters(
                1500.0, 2.7, 1.2, 1.5, 0.5, 2500.0,
                80000.0, 80000.0, 0.6, 4000.0, 8000.0);
        RoadCondition road = new RoadCondition(0.9, 0.015);
        DrivingSchedule schedule = new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(4.99, new DriverInput(1.0, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(5.0, new DriverInput(0.0, 0.8, 0.0)),
                new DrivingSchedule.TimedInput(10.0, new DriverInput(0.0, 0.8, 0.0))));
        Scenario scenario = new Scenario("smoke", vehicle, road, schedule);
        SimulationConfig config = new SimulationConfig(
                0.01, new double[]{0.0, 2.5, 5.0, 7.5, 10.0}, 100);

        Simulator simulator = new Simulator(scenario, config, null);
        StateSequence sequence = simulator.run();

        List<VehicleState> steps = sequence.stepStates();
        System.out.println("Steps: " + steps.size());
        for (int i = 0; i < steps.size(); i += 100) {
            VehicleState s = steps.get(i);
            System.out.printf("t=%.2f x=%.2f y=%.2f vx=%.2f vy=%.2f ax=%.2f ay=%.2f yaw=%.3f r=%.3f%n",
                    s.time(), s.x(), s.y(), s.vx(), s.vy(), s.ax(), s.ay(), s.yaw(), s.yawRate());
        }
        VehicleState last = steps.get(steps.size() - 1);
        System.out.println("Final: " + last);

        double maxAccel = 0.0;
        for (VehicleState s : steps) {
            double a = Math.hypot(s.ax(), s.ay());
            if (a > maxAccel) maxAccel = a;
        }
        System.out.println("Max accel: " + maxAccel + " (limit: " + 0.9 * 9.81 + ")");
        assertTrue(maxAccel <= 0.9 * 9.81 * 1.01, "Acceleration exceeds limit");
    }
}
