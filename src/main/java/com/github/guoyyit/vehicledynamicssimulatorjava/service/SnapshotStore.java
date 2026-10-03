package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.Checkpoint;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;

import java.util.List;
import java.util.Optional;

/**
 * Persistent store for scenario definitions and simulation checkpoints.
 */
public interface SnapshotStore {

    void saveScenario(Scenario scenario);

    Optional<Scenario> findScenario(String scenarioId);

    List<Scenario> listScenarios();

    void save(Checkpoint checkpoint);

    Optional<Checkpoint> findLatest(String scenarioId);

    List<Checkpoint> list(String scenarioId);

    void delete(String scenarioId);
}
