package com.github.guoyyit.vehicledynamicssimulatorjava.model;

/**
 * One driving input sample, applied from {@code time} until the next sample
 * (zero-order hold).
 *
 * @param time             sample time since scenario start [s]
 * @param longitudinalAccel commanded longitudinal acceleration [m/s^2],
 *                          positive = drive, negative = brake
 * @param steerAngle       front wheel steering angle [rad], positive = left
 */
public record DrivingInput(double time, double longitudinalAccel, double steerAngle) {
}
