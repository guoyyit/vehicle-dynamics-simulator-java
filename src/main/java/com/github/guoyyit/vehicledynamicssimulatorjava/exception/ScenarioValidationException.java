package com.github.guoyyit.vehicledynamicssimulatorjava.exception;

/**
 * Thrown when a scenario, its parameters or configuration are invalid.
 */
public class ScenarioValidationException extends SimulationException {
    public ScenarioValidationException(String message) {
        super(message);
    }
}
