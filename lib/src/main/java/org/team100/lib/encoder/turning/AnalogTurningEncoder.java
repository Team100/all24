package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 */
public class AnalogTurningEncoder implements Encoder100<Angle100> {
    private final Logger m_logger;
    private final String m_name;
    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;

    private Double prevAngle = null;
    private Double prevTime = null;

    /**
     * @param name        may not start with a slash
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param gearRatio
     * @param drive       polarity
     */
    public AnalogTurningEncoder(
            String name,
            Logger parent,
            int channel,
            double inputOffset,
            double gearRatio,
            EncoderDrive drive) {
        m_name = Names.append(name, this);
        m_logger = parent.child(this);
        m_input = new AnalogInput(channel);
        m_encoder = new AnalogEncoder(m_input);
        m_encoder.setPositionOffset(inputOffset);
        switch (drive) {
            case DIRECT:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / gearRatio);
                break;
            case INVERSE:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / (-1.0 * gearRatio));
                break;
        }
        m_logger.log(Level.DEBUG, "channel", m_encoder.getChannel());
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
        m_encoder.close();
    }

    //////////////////////////////////////////////

    private double getPositionRad() {
        // this should be fast, need not be cached.
        double positionRad = m_encoder.getDistance();
        m_logger.logDouble(Level.DEBUG, "position (rad)",()-> positionRad);
        m_logger.logDouble(Level.DEBUG, "position (turns)",()-> m_encoder.get());
        m_logger.logDouble(Level.DEBUG, "position (absolute)", ()->m_encoder.getAbsolutePosition());
        m_logger.logDouble(Level.DEBUG, "position (volts)",()-> m_input.getVoltage());
        return positionRad;
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
        m_logger.logDouble(Level.DEBUG, "rate (rad)s)",()-> rateRad_S);
        return rateRad_S;
    }
}
