package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;

/**
 * Absolute rotary position sensor using duty cycle input.
 */
public abstract class DutyCycleRotaryPositionSensor implements RotaryPositionSensor {
    private static final double kTwoPi = 2.0 * Math.PI;

    private final Logger m_logger;
    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    private final int m_frequencyThreshold;
    private final double m_positionOffset;
    private final EncoderDrive m_drive;


    protected abstract double m_sensorMin();
    protected abstract double m_sensorMax();

    private Double m_prevAngleRad = null;
    private Double m_prevTimeS = null;

    protected DutyCycleRotaryPositionSensor(
            Logger parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        m_logger = parent.child(this);
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_positionOffset = Util.inRange(inputOffset, 0.0, 1.0);
        m_drive = drive;
        m_frequencyThreshold = 1000;
    }

    @Override
    public OptionalDouble getPositionRad() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        double positionRad = getRad();
        m_logger.logInt(Level.TRACE, "channel", m_dutyCycle::getSourceChannel);
        m_logger.logDouble(Level.TRACE, "position (rad)", () -> positionRad);

        return OptionalDouble.of(positionRad);
    }

    @Override
    public OptionalDouble getRateRad_S() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(getRad_S());
    }

    @Override
    public void reset() {
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    @Override
    public void close() {
        m_dutyCycle.close();
        m_digitalInput.close();
    }

    ////////////////////////////////////

    private boolean isConnected() {
        return m_dutyCycle.getFrequency() > m_frequencyThreshold;
    }

    /** map to full [0,1] */
    private double mapSensorRange(double pos) {
        // map sensor range
        if (pos < m_sensorMin()) {
            pos = m_sensorMin();
        }
        if (pos > m_sensorMax()) {
            pos = m_sensorMax();
        }
        pos = (pos - m_sensorMin()) / (m_sensorMax() - m_sensorMin());
        return pos;
    }

    /** Radians, [-pi, pi] */
    private double getRad() {
        double dutyCycle = m_dutyCycle.getOutput();
        m_logger.logDouble(Level.TRACE, "duty cycle", () -> dutyCycle);

        double posTurns = mapSensorRange(dutyCycle);
        m_logger.logDouble(Level.TRACE, "position (turns)", () -> posTurns);

        double turnsMinusOffset = posTurns - m_positionOffset;
        m_logger.logDouble(Level.TRACE, "position (turns-offset)", () -> turnsMinusOffset);

        switch (m_drive) {
            case DIRECT:
                return MathUtil.angleModulus(turnsMinusOffset * kTwoPi);
            case INVERSE:
                return MathUtil.angleModulus(-1.0 * turnsMinusOffset * kTwoPi);
            default:
                throw new IllegalArgumentException();
        }
    }

    /** this is just finite difference over one time step. noisy! */
    private double getRad_S() {
        double angle = getRad();
        double time = Timer.getFPGATimestamp();
        if (m_prevAngleRad == null) {
            m_prevAngleRad = angle;
            m_prevTimeS = time;
            return 0;
        }
        double dx = angle - m_prevAngleRad;
        double dt = time - m_prevTimeS;

        m_prevAngleRad = angle;
        m_prevTimeS = time;

        double rateRad_S = dx / dt;
        m_logger.logDouble(Level.TRACE, "rate (rad_s)", () -> rateRad_S);
        return rateRad_S;
    }
}
