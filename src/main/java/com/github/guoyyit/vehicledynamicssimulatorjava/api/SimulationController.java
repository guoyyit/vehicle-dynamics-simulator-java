package com.github.guoyyit.vehicledynamicssimulatorjava.api;

import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationResult;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SimulationService;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/**
 * REST entry points for running and resuming scenario simulations.
 */
@RestController
@RequestMapping("/api/simulation")
public class SimulationController {

    private final SimulationService service;

    public SimulationController(SimulationService service) {
        this.service = service;
    }

    @PostMapping("/{runId}/run")
    public SimulationResult run(@PathVariable String runId, @RequestBody RunRequest request) {
        return service.runScenario(runId, request.scenario(), request.config());
    }

    @PostMapping("/{runId}/resume")
    public SimulationResult resume(@PathVariable String runId, @RequestBody RunRequest request) {
        return service.resumeScenario(runId, request.scenario(), request.config());
    }

    @ExceptionHandler(SimulationException.class)
    public ResponseEntity<String> handleSimulationError(SimulationException e) {
        return ResponseEntity.status(HttpStatus.UNPROCESSABLE_ENTITY).body(e.getMessage());
    }

    public record RunRequest(Scenario scenario, SimulationConfig config) {
    }
}
