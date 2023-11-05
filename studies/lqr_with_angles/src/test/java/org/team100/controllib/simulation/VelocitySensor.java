package org.team100.controllib.simulation;

import java.util.Random;

import org.team100.controllib.estimator.NewBitemporalEstimatorController;
import org.team100.controllib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * For test simulation only. Adds gaussian noise.
 *
 * this is modeled after the sparkmax.
 * observes position continuously
 * records a measurement every 500us and remembers the past N, say, 10?
 * uses the front and back of the window to compute velocity as the secant of
 * position.
 * 
 * currently this just does 20ms windows
 * TODO: actually do the correct thing.
 */
public class VelocitySensor {
    private static final long measurementPeriodUs = 500;
    private static final long messagePeriodUs = 20000;

    private final DoubleIntegratorRotary1D system;
    private final NewBitemporalEstimatorController<N2, N1, N2> estimator;
    private final Random m_random;
    private final double stdev;

    double measurementValue;
    long measurementTimestampUs;
    double messageValue;
    long messageTimestampUs;

    public VelocitySensor(
            DoubleIntegratorRotary1D system,
            NewBitemporalEstimatorController<N2, N1, N2> estimator) {
        this.system = system;
        this.estimator = estimator;
        m_random = new Random();
        stdev = Math.sqrt(system.v().Kxx.get(1, 1));
    }

    public void step(CompleteState state) {
        long timeUs = state.systemTimeMicrosec;
        double currentTime = state.actualTimeSec();

        if (timeUs > measurementTimestampUs + measurementPeriodUs) {
            // take a new measurement
            measurementValue = state.actualVelocity;
            // add measurement noise
            measurementValue += m_random.nextGaussian(0.0, stdev / 100);
            measurementTimestampUs = timeUs;
        }

        if (timeUs > messageTimestampUs + messagePeriodUs) {
            // send a measurement over can bus
            messageValue = measurementValue;
            messageTimestampUs = timeUs;
            // let the rio see the one that we sent
            state.observedVelocity = messageValue;
            state.velocityObservationTimeSec = currentTime;
            estimator.acceptMeasurement(timeUs, currentTime, system.velocity(messageValue));
        }
    }

}
