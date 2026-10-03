package com.github.guoyyit.vehicledynamicssimulatorjava.web;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.github.guoyyit.vehicledynamicssimulatorjava.TestScenarios;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DriverInput;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.DrivingSchedule;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.RoadCondition;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.Scenario;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.SimulationConfig;
import com.github.guoyyit.vehicledynamicssimulatorjava.model.VehicleParameters;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.FileSnapshotStore;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.ScenarioRepository;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.ScenarioService;
import com.github.guoyyit.vehicledynamicssimulatorjava.service.SnapshotStore;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.setup.MockMvcBuilders;

import java.nio.file.Path;
import java.util.List;

import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ScenarioControllerTest {

    @TempDir
    Path tempDir;

    private MockMvc mockMvc;
    private ObjectMapper objectMapper;

    @BeforeEach
    void setUp() {
        SnapshotStore store = new FileSnapshotStore(tempDir);
        ScenarioRepository repository = new ScenarioRepository(store);
        ScenarioService service = new ScenarioService(repository, store);
        ScenarioController controller = new ScenarioController(service);
        mockMvc = MockMvcBuilders.standaloneSetup(controller).build();
        objectMapper = new ObjectMapper();
    }

    @Test
    void createScenarioReturnsId() throws Exception {
        Scenario scenario = TestScenarios.scenario("web-1",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());

        mockMvc.perform(post("/api/scenarios")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(objectMapper.writeValueAsString(scenario)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.scenarioId").isNotEmpty())
                .andExpect(jsonPath("$.status").value("CREATED"));
    }

    @Test
    void runScenarioReturnsStateSequence() throws Exception {
        Scenario scenario = TestScenarios.scenario("web-2",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = createScenario(scenario);

        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);
        mockMvc.perform(post("/api/scenarios/" + id + "/run")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(objectMapper.writeValueAsString(config)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.scenarioId").value(id))
                .andExpect(jsonPath("$.stepStates").isArray())
                .andExpect(jsonPath("$.stepStates[0].time").value(0.0))
                .andExpect(jsonPath("$.queryStates").isArray());
    }

    @Test
    void queryStateReturnsStateAtTime() throws Exception {
        Scenario scenario = TestScenarios.scenario("web-3",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = createScenario(scenario);
        SimulationConfig config = TestScenarios.config(0.01, 0.0, 5.0, 10.0);
        mockMvc.perform(post("/api/scenarios/" + id + "/run")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(config)));

        mockMvc.perform(get("/api/scenarios/" + id + "/states")
                        .param("time", "5.0"))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.scenarioId").value(id))
                .andExpect(jsonPath("$.state.time").value(5.0));
    }

    @Test
    void invalidScenarioReturnsBadRequest() throws Exception {
        Scenario scenario = new Scenario("web-invalid",
                new VehicleParameters(-1.0, 2.7, 1.2, 1.5, 0.5, 2500.0,
                        80000.0, 80000.0, 0.6, 4000.0, 8000.0),
                TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());

        mockMvc.perform(post("/api/scenarios")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(objectMapper.writeValueAsString(scenario)))
                .andExpect(status().isBadRequest());
    }

    @Test
    void statusEndpointReturnsStatus() throws Exception {
        Scenario scenario = TestScenarios.scenario("web-4",
                TestScenarios.defaultVehicle(), TestScenarios.highFrictionRoad(),
                TestScenarios.accelerateBrakeSchedule());
        String id = createScenario(scenario);

        mockMvc.perform(get("/api/scenarios/" + id))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.scenarioId").value(id))
                .andExpect(jsonPath("$.status").value("CREATED"));
    }

    private String createScenario(Scenario scenario) throws Exception {
        String response = mockMvc.perform(post("/api/scenarios")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(objectMapper.writeValueAsString(scenario)))
                .andExpect(status().isOk())
                .andReturn().getResponse().getContentAsString();
        return objectMapper.readTree(response).get("scenarioId").asText();
    }
}
