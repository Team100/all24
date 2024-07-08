package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoDriveEncoder implements IncrementalBareEncoder {
    private final Logger m_logger;
    private final CANSparkMotor m_motor;

    public NeoDriveEncoder(
            Logger parent,
            CANSparkMotor motor) {
        m_logger = parent.child(this);
        m_motor = motor;
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

    @Override
    public OptionalDouble getPositionRad() {
        // raw position is in rotations
        // this is fast so we don't need to cache it
        double motorPositionRev = m_motor.getPositionRot();
        double positionRad = motorPositionRev * 2 * Math.PI;
        m_logger.logDouble(Level.TRACE, "motor position (rev)", ()->motorPositionRev);
        m_logger.logDouble(Level.TRACE, "position (rad)",()-> positionRad);
        return OptionalDouble.of(positionRad);
    }

    @Override
    public OptionalDouble getVelocityRad_S() {
        // raw velocity is in RPM
        // this is fast so we don't need to cache it
        double velocityRad_S = m_motor.getRateRPM() * 2 * Math.PI / 60;
        m_logger.logDouble(Level.TRACE, "velocity (rad_s)", ()->velocityRad_S);
        return OptionalDouble.of(velocityRad_S);
    }
}
