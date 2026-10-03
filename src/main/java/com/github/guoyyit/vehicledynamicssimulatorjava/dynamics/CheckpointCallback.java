package com.github.guoyyit.vehicledynamicssimulatorjava.dynamics;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

/**
 * Callback invoked whenever a checkpoint is reached during integration.
 */
@FunctionalInterface
public interface CheckpointCallback {
    void onCheckpoint(int stepIndex, double time, VehicleState state);
}
