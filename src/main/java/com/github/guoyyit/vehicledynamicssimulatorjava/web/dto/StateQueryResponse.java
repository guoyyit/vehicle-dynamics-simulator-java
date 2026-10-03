package com.github.guoyyit.vehicledynamicssimulatorjava.web.dto;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

/**
 * Response for a single state query.
 */
public record StateQueryResponse(String scenarioId, VehicleState state) {
}
