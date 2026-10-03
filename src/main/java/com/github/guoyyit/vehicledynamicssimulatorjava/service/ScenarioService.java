package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.Checkpoint;
import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.CheckpointCallback;
import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.Simulator;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.ScenarioValidationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Orchestrates scenario creation, execution and recovery. The service is
 * thread-safe: each run operates on an independent {@link Simulator} instance
 * and all shared state is held in concurrent maps.
 */
public class ScenarioService {

    private final ScenarioRepository repository;
    private final SnapshotStore store;
    private final Map<String, StateSequence> lastRuns = new ConcurrentHashMap<>();

    public ScenarioService(ScenarioRepository repository, SnapshotStore store) {
        this.repository = repository;
        this.store = store;
    }

    /**
     * Creates and persists a new scenario.
     */
    public String createScenario(Scenario scenario) {
        String id = scenario.id() == null || scenario.id().isBlank()
                ? UUID.randomUUID().toString() : scenario.id();
        Scenario withId = new Scenario(id, scenario.vehicle(), scenario.road(), scenario.schedule());
        withId.validate();
        store.saveScenario(withId);
        repository.save(withId);
        return id;
    }

    /**
     * Runs a scenario from the initial state and checkpoints periodically.
     */
    public StateSequence runScenario(String scenarioId, SimulationConfig config) {
        Scenario scenario = loadScenario(scenarioId);
        repository.updateStatus(scenarioId, ScenarioRepository.Status.RUNNING);
        try {
            CheckpointCallback callback = (step, time, state) ->
                    store.save(new Checkpoint(scenarioId, step, time, state));
            Simulator simulator = new Simulator(scenario, config, callback);
            StateSequence sequence = simulator.run();
            repository.updateStatus(scenarioId, ScenarioRepository.Status.COMPLETED);
            lastRuns.put(scenarioId, sequence);
            return sequence;
        } catch (RuntimeException e) {
            repository.updateStatus(scenarioId, ScenarioRepository.Status.FAILED);
            throw e;
        }
    }

    /**
     * Resumes a scenario from its latest checkpoint and continues to the
     * horizon. The returned sequence starts at the checkpoint step.
     */
    public StateSequence resumeScenario(String scenarioId, SimulationConfig config) {
        Scenario scenario = loadScenario(scenarioId);
        Checkpoint checkpoint = store.findLatest(scenarioId)
                .orElseThrow(() -> new SimulationException(
                        "No checkpoint found for scenario: " + scenarioId));
        repository.updateStatus(scenarioId, ScenarioRepository.Status.RUNNING);
        try {
            CheckpointCallback callback = (step, time, state) ->
                    store.save(new Checkpoint(scenarioId, step, time, state));
            Simulator simulator = Simulator.resume(scenario, config, checkpoint, callback);
            StateSequence sequence = simulator.run();
            repository.updateStatus(scenarioId, ScenarioRepository.Status.COMPLETED);
            lastRuns.put(scenarioId, sequence);
            return sequence;
        } catch (RuntimeException e) {
            repository.updateStatus(scenarioId, ScenarioRepository.Status.FAILED);
            throw e;
        }
    }

    /**
     * Queries the state at the given time from the last run of the scenario.
     */
    public VehicleState queryState(String scenarioId, double time) {
        StateSequence sequence = lastRuns.get(scenarioId);
        if (sequence == null) {
            throw new SimulationException("No run found for scenario: " + scenarioId);
        }
        return sequence.stateAt(time);
    }

    /**
     * Returns the current status of a scenario.
     */
    public ScenarioRepository.Status statusOf(String scenarioId) {
        return repository.statusOf(scenarioId);
    }

    private Scenario loadScenario(String scenarioId) {
        Scenario scenario = repository.findById(scenarioId)
                .orElseThrow(() -> new ScenarioValidationException(
                        "Scenario not found: " + scenarioId));
        scenario.validate();
        return scenario;
    }
}
