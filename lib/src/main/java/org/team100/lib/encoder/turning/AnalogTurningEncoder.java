package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;

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
public class AnalogTurningEncoder implements Encoder100<Angle100> {
    private final Logger m_logger;
    private final AnalogInput m_input;
    private final double m_positionOffset;
    private final double m_distancePerRotation;

    private Double prevAngle = null;
    private Double prevTime = null;

    /**
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param gearRatio
     * @param drive       polarity
     */
    public AnalogTurningEncoder(
            Logger parent,
            int channel,
            double inputOffset,
            double gearRatio,
            EncoderDrive drive) {
        m_logger = parent.child(this);
        m_input = new AnalogInput(channel);
        m_positionOffset = MathUtil.clamp(inputOffset, 0.0, 1.0);
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
        m_logger.logInt(Level.TRACE, "channel", m_input::getChannel);
    }

    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(getPositionRad());
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
        m_input.close();
    }

    //////////////////////////////////////////////

    private double getPositionRad() {
        // this should be fast, need not be cached.
        double positionRad = getDistance();
        m_logger.logDouble(Level.TRACE, "position (rad)", () -> positionRad);
        m_logger.logDouble(Level.TRACE, "position (turns+offset)", this::get);
        m_logger.logDouble(Level.TRACE, "position (turns)", this::getAbsolutePosition);
        m_logger.logDouble(Level.TRACE, "position (volts)", m_input::getVoltage);
        return positionRad;
    }

    /** in turns [0,1] */
    private double getAbsolutePosition() {
        return m_input.getVoltage() / RobotController.getVoltage5V();
    }

    private double getDistancePerRotation() {
        return m_distancePerRotation;
    }

    /** In turns but with offset, range is outside [0,1] */
    private double get() {
        double posTurns = getAbsolutePosition();
        return posTurns - m_positionOffset;
    }

    private double getDistance() {
        return get() * getDistancePerRotation();
    }

    /**
     * This is *just* the discrete difference looking back one time period, so it
     * will be very noisy.
     */
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
        m_logger.logDouble(Level.TRACE, "rate (rad)s)", () -> rateRad_S);
        return rateRad_S;
    }
}
