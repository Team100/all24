package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;

/**
 * Encoder using the AMS 5048 PWM output through a RoboRIO DIO port.
 * 
 * This is a near-copy of AnalogTurningEncoder; sadly the underlying WPI types
 * have no common parent, so there's some duplication here.
 * 
 * The 2025 update changed the DutyCycleEncoder class a lot, so i copied some of
 * the old methods here.
 * 
 * TODO: this should be further cleaned up once the 2025 transition work is
 * done.
 * 
 * TODO: combine with DutyCycleEncoder100.
 * 
 * TODO: remove gear ratio, it's always 1.
 */
public class DutyCycleTurningEncoder implements Encoder100<Angle100> {
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

    public DutyCycleTurningEncoder(
            Logger parent,
            int channel,
            double inputOffset,
            double gearRatio,
            EncoderDrive drive) {
        m_logger = parent.child(this);
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_positionOffset = inputOffset;
        switch (drive) {
            case DIRECT:
                m_distancePerRotation = 2.0 * Math.PI / gearRatio;
                break;
            case INVERSE:
                m_distancePerRotation = 2.0 * Math.PI / (-1.0 * gearRatio);
                break;
            default:
                throw new IllegalArgumentException();
        }
        m_frequencyThreshold = 1000;
    }

    @Override
    public OptionalDouble getPosition() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(getPositionRad());
    }

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
        double positionRad = getDistance();
        m_logger.logInt(Level.TRACE, "channel", m_dutyCycle::getSourceChannel);
        m_logger.logDouble(Level.TRACE, "position (rad) ROBOT USES THIS (CCW POSITIVE)", () -> positionRad);
        m_logger.logDouble(Level.TRACE, "position (turns-offset) USE FOR OFFSETS", this::get);
        m_logger.logDouble(Level.TRACE, "position (absolute)", this::getAbsolutePosition);
        m_logger.logDouble(Level.TRACE, "Wrapped position rads (absolute)", () -> MathUtil.angleModulus(positionRad));
        return positionRad;
    }

    /** this is just finite difference over one time step. noisy! */
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

        double rateRad_S = dx / dt;
        m_logger.logDouble(Level.TRACE, "rate (rad_s)", () -> rateRad_S);
        return rateRad_S;
    }
}
