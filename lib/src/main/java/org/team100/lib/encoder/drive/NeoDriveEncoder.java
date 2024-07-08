package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalLinearEncoder;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoDriveEncoder implements IncrementalLinearEncoder {
    private final Logger m_logger;
    private final NeoCANSparkMotor m_motor;
    private final double m_distancePerTurn;

    public NeoDriveEncoder(
            Logger parent,
            NeoCANSparkMotor motor,
            double distancePerTurn) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters. */
    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(getPositionM());
    }

    /** Velocity in meters/sec. */
    @Override
    public OptionalDouble getRate() {
        return OptionalDouble.of(getVelocityM_S());
    }

    // /** Position in meters. */
    // @Override
    // public void setPosition(double positionM) {
    //     double motorPositionRev = positionM / m_distancePerTurn;
    //     m_motor.setEncoderPosition(motorPositionRev);
    // }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        //
    }

    //////////////////////////////////

    private double getPositionM() {
        // raw position is in rotations
        // this is fast so we don't need to cache it
        double motorPositionRev = m_motor.getPositionRot();
        double positionM = motorPositionRev * m_distancePerTurn;
        m_logger.logDouble(Level.TRACE, "motor position (rev)", ()->motorPositionRev);
        m_logger.logDouble(Level.TRACE, "position (m)",()-> positionM);
        return positionM;
    }

    private double getVelocityM_S() {
        // raw velocity is in RPM
        // this is fast so we don't need to cache it
        double velocityM_S = m_motor.getRateRPM() * m_distancePerTurn / 60;
        m_logger.logDouble(Level.TRACE, "velocity (m_s)", ()->velocityM_S);
        return velocityM_S;
    }
}
