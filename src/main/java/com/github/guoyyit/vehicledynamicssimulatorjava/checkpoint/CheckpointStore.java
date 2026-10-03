package com.github.guoyyit.vehicledynamicssimulatorjava.checkpoint;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Thread-safe in-memory store of the latest saved state per scenario run,
 * used for failure recovery. Keys are run identifiers.
 */
public class CheckpointStore {

    private final ConcurrentHashMap<String, VehicleState> latestByRun = new ConcurrentHashMap<>();

    public void save(String runId, VehicleState state) {
        latestByRun.put(runId, state);
    }

    public Optional<VehicleState> latest(String runId) {
        return Optional.ofNullable(latestByRun.get(runId));
    }

    public void clear(String runId) {
        latestByRun.remove(runId);
    }
}
