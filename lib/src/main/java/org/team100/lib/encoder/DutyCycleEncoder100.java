package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;

/**
 * Duty cycle encoder such as the AMS 5048.
 * 
 * The 2025 update changed the DutyCycleEncoder class a lot, so i copied some of
 * the old methods here.
 * 
 * TODO: this should be further cleaned up once the 2025 transition work is
 * done.
 * 
 * TODO: combine with DutyCycleTurningEncoder
 */
public class DutyCycleEncoder100 implements Encoder100<Distance100> {
    private final Logger m_logger;
    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    private final int m_frequencyThreshold;
    private final double m_positionOffset;
    private final double m_distancePerRotation;
    // TODO: use these to fix https://github.com/Team100/all24/issues/383
    // these are within [0,1]
    private double m_sensorMin = 0.0;
    private double m_sensorMax = 1.0;

    private Double prevAngle = null;
    private Double prevTime = null;

    private boolean m_reversed;

    /**
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param reversed    polarity
     */
    public DutyCycleEncoder100(
            Logger parent,
            int channel,
            double inputOffset,
            boolean reversed) {
        m_reversed = reversed;
        m_logger = parent.child(this);
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_distancePerRotation = 2 * Math.PI;
        m_positionOffset = inputOffset;
        m_frequencyThreshold = 1000;
    }

    @Override
    public OptionalDouble getPosition() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        if (m_reversed) {
            return OptionalDouble.of(-1.0 * (getAbsolutePosition() - m_positionOffset)
                    * m_distancePerRotation);
        }
        return OptionalDouble.of(
                (getAbsolutePosition() - m_positionOffset) * m_distancePerRotation);
    }

    /**
     * Current rate in rad/s.
     * 
     * This is simply the backward finite difference over one time step.
     * 
     * As such, it is likely to be very noisy.
     * 
     * Use a simple filter if you want a lagged, smoother measurement.
     * 
     * Use a Kalman filter if you can, to reduce the lag.
     */
    @Override
    public OptionalDouble getRate() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(getRateRad_S());
    }

    @Override
    public void reset() {
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    public void close() {
        m_dutyCycle.close();
        m_digitalInput.close();
    }

    //////////////////////////////////////////////

    private boolean isConnected() {
        return m_dutyCycle.getFrequency() > m_frequencyThreshold;
    }

    /** map to full [0,1] */
    private double mapSensorRange(double pos) {
        // map sensor range
        if (pos < m_sensorMin) {
            pos = m_sensorMin;
        }
        if (pos > m_sensorMax) {
            pos = m_sensorMax;
        }
        pos = (pos - m_sensorMin) / (m_sensorMax - m_sensorMin);
        return pos;
    }

    /** return turns [0,1] */
    private double getAbsolutePosition() {
        return mapSensorRange(m_dutyCycle.getOutput());
    }

    /**
     * return turns minus offset, can be outside [0,1]
     * TODO: modulus to [0,1]
     */
    private double get() {
        double dutyCycle = m_dutyCycle.getOutput();
        double posTurns = mapSensorRange(dutyCycle);
        return posTurns - m_positionOffset;
    }

    private double getDistance() {
        return get() * m_distancePerRotation;
    }

    private double getPositionRad() {
        // should be fast, no cache needed.
        double posRad = getDistance();
        m_logger.logDouble(Level.TRACE, "position (rad)", () -> posRad);
        m_logger.logDouble(Level.TRACE, "position (turns-offset)", this::get);
        m_logger.logDouble(Level.TRACE, "position (absolute)", this::getAbsolutePosition);
        return posRad;
    }

    /** finite difference == noise! */
    private double getRateRad_S() {
        double angle = getPositionRad();
        double time = Timer.getFPGATimestamp();
        if (prevAngle == null) {
            prevAngle = angle;
            prevTime = time;
            return 0;
        }
        double dx = angle - prevAngle;
        double dt = time - prevTime;

        prevAngle = angle;
        prevTime = time;

        return dx / dt;
    }
}
