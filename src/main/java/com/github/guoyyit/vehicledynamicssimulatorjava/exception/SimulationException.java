package com.github.guoyyit.vehicledynamicssimulatorjava.exception;

/**
 * Raised for illegal input, out-of-range parameters or numerical divergence.
 * A scenario that hits this exception is terminated and marked FAILED; no
 * partial result is returned.
 */
public class SimulationException extends RuntimeException {

    public SimulationException(String message) {
        super(message);
    }

    public SimulationException(String message, Throwable cause) {
        super(message, cause);
    }
}
