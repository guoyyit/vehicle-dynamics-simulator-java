package com.github.guoyyit.vehicledynamicssimulatorjava.web.dto;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingSchedule;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;

/**
 * Request body for creating a scenario.
 */
public record ScenarioRequest(
        VehicleParameters vehicle,
        RoadCondition road,
        DrivingSchedule schedule) {
}
