package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 * 
 * The 2025 changes to AnalogEncoder changed the way the zeros are calculated,
 * so I imported the old methods here for now.
 * 
 * TODO: the gear ratio is always 1, so streamline this a bit.
 */
public class AnalogTurningEncoder implements RotaryPositionSensor {
    private static final double kTwoPi = 2.0 * Math.PI;
    private final Logger m_logger;
    private final AnalogInput m_input;
    private final double m_positionOffset;
    private final EncoderDrive m_drive;

    private Double m_prevAngleRad = null;
    private Double m_prevTimeS = null;

    /**
     * @param logger 
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param drive       polarity
     */
    public AnalogTurningEncoder(
            Logger parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        m_logger = parent.child(this);
        m_input = new AnalogInput(channel);
        m_positionOffset = Util.inRange(inputOffset, 0.0, 1.0);
        m_drive = drive;
    }

    @Override
    public OptionalDouble getPositionRad() {
        // this should be fast, need not be cached.
        double positionRad = getRad();
        m_logger.logInt(Level.TRACE, "channel", m_input::getChannel);
        m_logger.logDouble(Level.TRACE, "position (rad)", () -> positionRad);
        m_logger.logDouble(Level.TRACE, "position (turns-offset)", this::get);
        m_logger.logDouble(Level.TRACE, "position (turns)", this::getAbsolutePosition);
        m_logger.logDouble(Level.TRACE, "position (volts)", m_input::getVoltage);
        return OptionalDouble.of(positionRad);
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
    public OptionalDouble getRateRad_S() {
        double angleRad = getRad();
        double timeS = Timer.getFPGATimestamp();
        if (m_prevAngleRad == null) {
            m_prevAngleRad = angleRad;
            m_prevTimeS = timeS;
            return OptionalDouble.of(0);
        }
        double dxRad = MathUtil.angleModulus(angleRad - m_prevAngleRad);
        double dtS = timeS - m_prevTimeS;

        m_prevAngleRad = angleRad;
        m_prevTimeS = timeS;

        double rateRad_S = dxRad / dtS;
        m_logger.logDouble(Level.TRACE, "rate (rad)s)", () -> rateRad_S);
        return OptionalDouble.of(rateRad_S);

    }

    @Override
    public void reset() {
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    public void close() {
        m_input.close();
    }

    //////////////////////////////////////////////

    /** Turns [0, 1] */
    private double getAbsolutePosition() {
        return m_input.getVoltage() / RobotController.getVoltage5V();
    }

    /**
     * Turns minus offset, could be outside [0, 1]
     */
    private double get() {
        double posTurns = getAbsolutePosition();
        return posTurns - m_positionOffset;
    }

    /** Radians, [-pi, pi] */
    private double getRad() {
        double posTurnsMinusOffset = get();
        switch (m_drive) {
            case DIRECT:
                return MathUtil.angleModulus(posTurnsMinusOffset * kTwoPi);
            case INVERSE:
                return MathUtil.angleModulus(-1.0 * posTurnsMinusOffset * kTwoPi);
            default:
                throw new IllegalArgumentException();
        }

    }
}
