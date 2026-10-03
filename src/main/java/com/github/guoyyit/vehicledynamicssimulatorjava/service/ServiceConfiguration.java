package com.github.guoyyit.vehicledynamicssimulatorjava.service;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

import java.nio.file.Path;

/**
 * Spring configuration for the simulation service.
 */
@Configuration
public class ServiceConfiguration {

    @Bean
    public SnapshotStore snapshotStore(
            @Value("${simulator.snapshot-directory:${java.io.tmpdir}/vehicle-simulator-snapshots}") String directory) {
        return new FileSnapshotStore(Path.of(directory));
    }

    @Bean
    public ScenarioRepository scenarioRepository(SnapshotStore store) {
        return new ScenarioRepository(store);
    }

    @Bean
    public ScenarioService scenarioService(ScenarioRepository repository, SnapshotStore store) {
        return new ScenarioService(repository, store);
    }
}
