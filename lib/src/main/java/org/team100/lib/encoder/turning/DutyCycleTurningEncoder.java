package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

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
    private final Telemetry.Logger t;
    private final DutyCycleEncoder m_encoder;
    private final String m_name;

    private Double prevAngle = null;
    private Double prevTime = null;

    public DutyCycleTurningEncoder(
            String name,
            int channel,
            double inputOffset,
            double gearRatio,
            EncoderDrive drive) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
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

        t.log(Level.DEBUG,  "channel", m_encoder.getSourceChannel());
    }

    @Override
    public OptionalDouble getPosition() {
        if (!m_encoder.isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_encoder.getSourceChannel()));
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(getPositionRad());
    }

    @Override
    public OptionalDouble getRate() {
        if (!m_encoder.isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_encoder.getSourceChannel()));
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
        m_encoder.close();
    }

    ////////////////////////////////////

    private double getPositionRad() {
        double positionRad = m_encoder.getDistance();
        t.logDouble(Level.DEBUG,  "position (rad) ROBOT USES THIS (CCW POSITIVE)",()-> positionRad);
        t.logDouble(Level.DEBUG,  "position (turns) USE FOR OFFSETS",()-> m_encoder.get());
        t.logDouble(Level.DEBUG,  "position (absolute)", ()->m_encoder.getAbsolutePosition());
        t.logDouble(Level.DEBUG,  "Wrapped position rads (absolute)", ()->MathUtil.angleModulus(positionRad));
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
        t.logDouble(Level.DEBUG,  "rate (rad_s)",()-> rateRad_S);
        return rateRad_S;
    }
}
