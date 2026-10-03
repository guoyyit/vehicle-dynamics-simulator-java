package com.github.guoyyit.vehicledynamicssimulatorjava.exception;

/**
 * Base exception for all simulation-related failures.
 */
public class SimulationException extends RuntimeException {
    public SimulationException(String message) {
        super(message);
    }

    public SimulationException(String message, Throwable cause) {
        super(message, cause);
    }
}
