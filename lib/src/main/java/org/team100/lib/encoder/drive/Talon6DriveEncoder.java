package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalLinearEncoder;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

public class Talon6DriveEncoder implements IncrementalLinearEncoder {
    private final Logger m_logger;
    private final Talon6Motor m_motor;
    private final double m_distancePerTurn;

    public Talon6DriveEncoder(
            Logger parent,
            Talon6Motor motor,
            double distancePerTurn) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters */
    @Override
    public OptionalDouble getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionM = motorPositionRev * m_distancePerTurn;
        m_logger.logDouble(Level.TRACE,  "motor position (rev)", ()->motorPositionRev);
        m_logger.logDouble(Level.TRACE,  "position (m)",()-> positionM);
        return OptionalDouble.of(positionM);
    }

    /** Velocity in meters/sec */
    @Override
    public OptionalDouble getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityM_S = motorVelocityRev_S * m_distancePerTurn;
        m_logger.logDouble(Level.TRACE,  "motor velocity (rev_s)", ()->motorVelocityRev_S);
        m_logger.logDouble(Level.TRACE,  "velocity (m_s)", ()->velocityM_S);
        return OptionalDouble.of(velocityM_S);
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        m_motor.close();
    }
}
