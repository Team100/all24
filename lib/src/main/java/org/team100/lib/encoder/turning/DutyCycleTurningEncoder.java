package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Encoder using the AMS 5048 PWM output through a RoboRIO DIO port.
 * 
 * This is a near-copy of AnalogTurningEncoder; sadly the underlying WPI types
 * have no common parent, so there's some duplication here.
 */
public class DutyCycleTurningEncoder implements Encoder100<Angle100> {
    private final Telemetry t = Telemetry.get();

    private final DutyCycleEncoder m_encoder;
    private final String m_name;

    private Double prevAngle = null;
    private Double prevTime = null;

    /** Current position, updated in periodic() */
    private double m_positionRad;
    /** Current velocity, updated in periodic() */
    private double m_rateRad_S;

    public DutyCycleTurningEncoder(
            String name,
            int channel,
            double inputOffset,
            double gearRatio,
            Drive drive) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_encoder = new DutyCycleEncoder(channel);
        m_encoder.setPositionOffset(inputOffset);
        switch (drive) {
            case DIRECT:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / gearRatio);
                break;
            case INVERSE:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / (-1.0 * gearRatio));
                break;
        }

        t.log(Level.DEBUG, m_name, "channel", m_encoder.getSourceChannel());
    }

    @Override
    public double getPosition() {
        return m_positionRad;
    }

    @Override
    public double getRate() {
        return m_rateRad_S;
    }

    @Override
    public void reset() {
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
        m_positionRad = 0;
    }

    @Override
    public void close() {
        m_encoder.close();
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rad) ROBOT USES THIS (COUNTER CLOCKWISE IS POSITIVE)",
                m_encoder.getDistance());
        t.log(Level.DEBUG, m_name, "position (turns) USE FOR OFFSETS", m_encoder.get());
        t.log(Level.DEBUG, m_name, "position (absolute)", m_encoder.getAbsolutePosition());
        t.log(Level.DEBUG, m_name, "Wrapped position rads (absolute)", getWrappedPosition());

    }

    ////////////////////////////////////

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

    private double getWrappedPosition() {
        return MathUtil.angleModulus(m_positionRad);
    }

}
