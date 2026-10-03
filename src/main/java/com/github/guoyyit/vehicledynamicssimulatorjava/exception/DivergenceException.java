package com.github.guoyyit.vehicledynamicssimulatorjava.exception;

/**
 * Thrown when the numerical integration diverges or produces non-finite values.
 */
public class DivergenceException extends SimulationException {
    public DivergenceException(String message) {
        super(message);
    }
}
