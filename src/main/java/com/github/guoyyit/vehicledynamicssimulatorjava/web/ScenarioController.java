package com.github.guoyyit.vehicledynamicssimulatorjava.web;

import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.exception.SimulationException;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.StateSequence;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleState;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.ScenarioRepository;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.ScenarioService;
import com.github.guoyyit.vehicledynamicssimulatorjava.web.dto.ScenarioRequest;
import com.github.guoyyit.vehicledynamicssimulatorjava.web.dto.ScenarioResponse;
import com.github.guoyyit.vehicledynamicssimulatorjava.web.dto.StateQueryResponse;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.server.ResponseStatusException;

import java.util.Map;

/**
 * REST API for the vehicle dynamics simulator.
 */
@RestController
@RequestMapping("/api/scenarios")
public class ScenarioController {

    private final ScenarioService service;

    public ScenarioController(ScenarioService service) {
        this.service = service;
    }

    @PostMapping
    public ScenarioResponse create(@RequestBody ScenarioRequest request) {
        Scenario scenario = new Scenario(null, request.vehicle(), request.road(), request.schedule());
        String id = service.createScenario(scenario);
        return new ScenarioResponse(id, ScenarioRepository.Status.CREATED.name());
    }

    @PostMapping("/{id}/run")
    public StateSequence run(@PathVariable("id") String id, @RequestBody SimulationConfig config) {
        return service.runScenario(id, config);
    }

    @PostMapping("/{id}/resume")
    public StateSequence resume(@PathVariable("id") String id, @RequestBody SimulationConfig config) {
        return service.resumeScenario(id, config);
    }

    @GetMapping("/{id}/states")
    public StateQueryResponse queryState(@PathVariable("id") String id, @RequestParam("time") double time) {
        VehicleState state = service.queryState(id, time);
        return new StateQueryResponse(id, state);
    }

    @GetMapping("/{id}")
    public Map<String, Object> status(@PathVariable("id") String id) {
        ScenarioRepository.Status status = service.statusOf(id);
        if (status == null) {
            throw new ResponseStatusException(HttpStatus.NOT_FOUND, "Scenario not found: " + id);
        }
        return Map.of("scenarioId", id, "status", status.name());
    }

    @ExceptionHandler(SimulationException.class)
    public ResponseStatusException handleValidation(SimulationException e) {
        return new ResponseStatusException(HttpStatus.BAD_REQUEST, e.getMessage());
    }
}
