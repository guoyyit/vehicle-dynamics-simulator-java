package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DriverInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingSchedule;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertThrows;

class ValidationTest {

    private ScenarioService service;

    @BeforeEach
    void setUp(@TempDir Path tempDir) {
        SnapshotStore store = new FileSnapshotStore(tempDir);
        ScenarioRepository repository = new ScenarioRepository(store);
        service = new ScenarioService(repository, store);
    }

    @Test
    void negativeMassThrows() {
        VehicleParameters vehicle = new VehicleParameters(
                -1.0, 2.7, 1.2, 1.5, 0.5, 2500.0,
                80000.0, 80000.0, 0.6, 4000.0, 8000.0);
        assertThrows(ScenarioValidationException.class, vehicle::validate);
    }

    @Test
    void zeroWheelbaseThrows() {
        VehicleParameters vehicle = new VehicleParameters(
                1500.0, 0.0, 1.2, 1.5, 0.5, 2500.0,
                80000.0, 80000.0, 0.6, 4000.0, 8000.0);
        assertThrows(ScenarioValidationException.class, vehicle::validate);
    }

    @Test
    void axleDistancesNotMatchingWheelbaseThrows() {
        VehicleParameters vehicle = new VehicleParameters(
                1500.0, 2.7, 1.0, 1.0, 0.5, 2500.0,
                80000.0, 80000.0, 0.6, 4000.0, 8000.0);
        assertThrows(ScenarioValidationException.class, vehicle::validate);
    }

    @Test
    void frictionCoefficientOutOfRangeThrows() {
        RoadCondition road = new RoadCondition(1.5, 0.015);
        assertThrows(ScenarioValidationException.class, road::validate);
    }

    @Test
    void zeroFrictionThrows() {
        RoadCondition road = new RoadCondition(0.0, 0.015);
        assertThrows(ScenarioValidationException.class, road::validate);
    }

    @Test
    void throttleOutOfRangeThrows() {
        DriverInput input = new DriverInput(1.5, 0.0, 0.0);
        assertThrows(ScenarioValidationException.class, input::validate);
    }

    @Test
    void brakeOutOfRangeThrows() {
        DriverInput input = new DriverInput(0.0, -0.1, 0.0);
        assertThrows(ScenarioValidationException.class, input::validate);
    }

    @Test
    void emptyScheduleThrows() {
        assertThrows(ScenarioValidationException.class,
                () -> new DrivingSchedule(List.of()));
    }

    @Test
    void nonIncreasingScheduleTimesThrow() {
        assertThrows(ScenarioValidationException.class, () -> new DrivingSchedule(List.of(
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.5, 0.0, 0.0)),
                new DrivingSchedule.TimedInput(0.0, new DriverInput(0.5, 0.0, 0.0)))));
    }

    @Test
    void steeringExceedsVehicleLimitThrows() {
        Scenario scenario = new Scenario("bad-steer",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                new DrivingSchedule(List.of(
                        new DrivingSchedule.TimedInput(0.0, new DriverInput(0.5, 0.0, 1.0)))));
        assertThrows(ScenarioValidationException.class, scenario::validate);
    }

    @Test
    void invalidStepSizeThrows() {
        assertThrows(ScenarioValidationException.class,
                () -> new SimulationConfig(0.0, new double[]{1.0}, 10));
    }

    @Test
    void emptyQueryTimesThrows() {
        assertThrows(ScenarioValidationException.class,
                () -> new SimulationConfig(0.01, new double[]{}, 10));
    }

    @Test
    void invalidCheckpointIntervalThrows() {
        assertThrows(ScenarioValidationException.class,
                () -> new SimulationConfig(0.01, new double[]{1.0}, 0));
    }

    @Test
    void createScenarioWithInvalidParamsThrows() {
        Scenario scenario = new Scenario("invalid",
                new VehicleParameters(-1.0, 2.7, 1.2, 1.5, 0.5, 2500.0,
                        80000.0, 80000.0, 0.6, 4000.0, 8000.0),
                TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        assertThrows(ScenarioValidationException.class,
                () -> service.createScenario(scenario));
    }

    @Test
    void runNonExistentScenarioThrows() {
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0);
        assertThrows(ScenarioValidationException.class,
                () -> service.runScenario("non-existent", config));
    }

    @Test
    void resumeWithoutCheckpointThrows() {
        Scenario scenario = TestScenarios.scenario("no-checkpoint",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        service.createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0);
        assertThrows(Exception.class,
                () -> service.resumeScenario("no-checkpoint", config));
    }
}
