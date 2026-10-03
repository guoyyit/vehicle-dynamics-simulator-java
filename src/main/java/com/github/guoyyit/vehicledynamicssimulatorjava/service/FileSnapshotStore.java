package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import com.github.guoyyit.vehicledynamicssimulatorjava.dynamics.Checkpoint;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

/**
 * File-based JSON snapshot store. Scenarios and checkpoints are persisted as
 * JSON files so that a service restart can resume from the latest checkpoint.
 */
public class FileSnapshotStore implements SnapshotStore {

    private static final String SCENARIOS_DIR = "scenarios";
    private static final String CHECKPOINTS_DIR = "checkpoints";

    private final Path rootDirectory;
    private final ObjectMapper objectMapper;

    public FileSnapshotStore(Path rootDirectory) {
        this.rootDirectory = rootDirectory;
        this.objectMapper = new ObjectMapper();
        try {
            Files.createDirectories(rootDirectory.resolve(SCENARIOS_DIR));
            Files.createDirectories(rootDirectory.resolve(CHECKPOINTS_DIR));
        } catch (IOException e) {
            throw new IllegalStateException("Failed to create snapshot directories", e);
        }
    }

    @Override
    public void saveScenario(Scenario scenario) {
        writeJson(scenarioFile(scenario.id()), scenario);
    }

    @Override
    public Optional<Scenario> findScenario(String scenarioId) {
        Path file = scenarioFile(scenarioId);
        if (!Files.exists(file)) {
            return Optional.empty();
        }
        return Optional.of(readJson(file, Scenario.class));
    }

    @Override
    public List<Scenario> listScenarios() {
        Path dir = rootDirectory.resolve(SCENARIOS_DIR);
        List<Scenario> scenarios = new ArrayList<>();
        try (Stream<Path> files = Files.list(dir)) {
            files.filter(Files::isRegularFile)
                    .filter(p -> p.getFileName().toString().endsWith(".json"))
                    .map(p -> readJson(p, Scenario.class))
                    .forEach(scenarios::add);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to list scenarios", e);
        }
        return scenarios;
    }

    @Override
    public void save(Checkpoint checkpoint) {
        Path dir = checkpointDir(checkpoint.scenarioId());
        try {
            Files.createDirectories(dir);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to create checkpoint directory", e);
        }
        writeJson(dir.resolve(checkpoint.stepIndex() + ".json"), checkpoint);
    }

    @Override
    public Optional<Checkpoint> findLatest(String scenarioId) {
        return list(scenarioId).stream().max(Comparator.comparingInt(Checkpoint::stepIndex));
    }

    @Override
    public List<Checkpoint> list(String scenarioId) {
        Path dir = checkpointDir(scenarioId);
        if (!Files.exists(dir)) {
            return List.of();
        }
        List<Checkpoint> checkpoints = new ArrayList<>();
        try (Stream<Path> files = Files.list(dir)) {
            files.filter(Files::isRegularFile)
                    .filter(p -> p.getFileName().toString().endsWith(".json"))
                    .map(p -> readJson(p, Checkpoint.class))
                    .forEach(checkpoints::add);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to list checkpoints", e);
        }
        checkpoints.sort(Comparator.comparingInt(Checkpoint::stepIndex));
        return checkpoints;
    }

    @Override
    public void delete(String scenarioId) {
        try {
            Files.deleteIfExists(scenarioFile(scenarioId));
            Path dir = checkpointDir(scenarioId);
            if (Files.exists(dir)) {
                try (Stream<Path> files = Files.walk(dir)) {
                    files.sorted(Comparator.reverseOrder()).forEach(p -> {
                        try {
                            Files.deleteIfExists(p);
                        } catch (IOException e) {
                            throw new IllegalStateException("Failed to delete " + p, e);
                        }
                    });
                }
            }
        } catch (IOException e) {
            throw new IllegalStateException("Failed to delete scenario " + scenarioId, e);
        }
    }

    private Path scenarioFile(String scenarioId) {
        return rootDirectory.resolve(SCENARIOS_DIR).resolve(scenarioId + ".json");
    }

    private Path checkpointDir(String scenarioId) {
        return rootDirectory.resolve(CHECKPOINTS_DIR).resolve(scenarioId);
    }

    private void writeJson(Path file, Object value) {
        try {
            objectMapper.writerWithDefaultPrettyPrinter().writeValue(file.toFile(), value);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to write " + file, e);
        }
    }

    private <T> T readJson(Path file, Class<T> type) {
        try {
            return objectMapper.readValue(file.toFile(), type);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to read " + file, e);
        }
    }
}
