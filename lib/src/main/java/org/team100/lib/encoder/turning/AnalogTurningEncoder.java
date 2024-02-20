package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 */
public class AnalogTurningEncoder implements Encoder100<Angle100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;

    private Double prevAngle = null;
    private Double prevTime = null;

    /** Current position, updated in periodic() */
    private double m_positionRad;
    /** Current velocity, updated in periodic() */
    private double m_rateRad_S;

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
            int channel,
            double inputOffset,
            double gearRatio,
            Drive drive) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
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
        t.log(Level.DEBUG, m_name, "channel", m_encoder.getChannel());
    }

    @Override
    public double getPosition() {
        return m_positionRad;
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
    public double getRate() {
        return m_rateRad_S;
    }

    @Override
    public void reset() {
        // ALERT!  @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want.  but this might be wrong
        // for some other reason
        // m_encoder.reset();
        m_positionRad = 0;
    }

    public void close() {
        m_input.close();
        m_encoder.close();
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rad)", m_encoder.getDistance());
        t.log(Level.DEBUG, m_name, "position (turns)", m_encoder.get());
        t.log(Level.DEBUG, m_name, "position (absolute)", m_encoder.getAbsolutePosition());
        t.log(Level.DEBUG, m_name, "position (volts)", m_input.getVoltage());
    }

    //////////////////////////////////////////////

    private void updatePosition() {
        m_positionRad = m_encoder.getDistance();
    }

    /** Update position before calling this. */
    private void updateRate() {
        double angle = m_positionRad;
        double time = Timer.getFPGATimestamp();
        if (prevAngle == null) {
            prevAngle = angle;
            prevTime = time;
            m_rateRad_S = 0;
            return;
        }
        double dx = angle - prevAngle;
        double dt = time - prevTime;

        prevAngle = angle;
        prevTime = time;

        m_rateRad_S = dx / dt;
    }
}
