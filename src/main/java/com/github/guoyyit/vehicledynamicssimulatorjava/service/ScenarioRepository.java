package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/**
 * In-memory repository of scenarios and their run status. Scenarios are loaded
 * from the persistent store on construction so that recovery works across
 * service restarts.
 */
public class ScenarioRepository {

    public enum Status {CREATED, RUNNING, COMPLETED, FAILED}

    private final Map<String, Scenario> scenarios = new ConcurrentHashMap<>();
    private final Map<String, Status> statuses = new ConcurrentHashMap<>();

    public ScenarioRepository() {
    }

    public ScenarioRepository(SnapshotStore store) {
        for (Scenario scenario : store.listScenarios()) {
            scenarios.put(scenario.id(), scenario);
            statuses.put(scenario.id(), Status.CREATED);
        }
    }

    public String save(Scenario scenario) {
        scenarios.put(scenario.id(), scenario);
        statuses.put(scenario.id(), Status.CREATED);
        return scenario.id();
    }

    public Optional<Scenario> findById(String id) {
        return Optional.ofNullable(scenarios.get(id));
    }

    public Status statusOf(String id) {
        return statuses.get(id);
    }

    public void updateStatus(String id, Status status) {
        statuses.put(id, status);
    }

    public List<Scenario> list() {
        return List.copyOf(scenarios.values());
    }

    public boolean exists(String id) {
        return scenarios.containsKey(id);
    }
}
