package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.motor.CANSparkMotor;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class CANSparkEncoder implements IncrementalBareEncoder {
    private final CANSparkMotor m_motor;
    // LOGGERS
    private final OptionalDoubleLogger m_log_position;
    private final OptionalDoubleLogger m_log_velocity;

    public CANSparkEncoder(
            SupplierLogger2 parent,
            CANSparkMotor motor) {
        SupplierLogger2 child = parent.child(this);
        m_motor = motor;
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.optionalDoubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    // /** Position in meters. */
    // @Override
    // public void setPosition(double positionM) {
    // double motorPositionRev = positionM / m_distancePerTurn;
    // m_motor.setEncoderPosition(motorPositionRev);
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

    /** Nearly cached. */
    @Override
    public OptionalDouble getPositionRad() {
        // raw position is in rotations
        // this is fast so we don't need to cache it
        double motorPositionRev = m_motor.getPositionRot();
        double positionRad = motorPositionRev * 2 * Math.PI;
        return OptionalDouble.of(positionRad);
    }

    /** Nearly cached. */
    @Override
    public OptionalDouble getVelocityRad_S() {
        // raw velocity is in RPM
        // this is fast so we don't need to cache it
        double velocityRad_S = m_motor.getRateRPM() * 2 * Math.PI / 60;
        return OptionalDouble.of(velocityRad_S);
    }

    @Override
    public void setEncoderPositionRad(double motorPositionRad) {
        m_motor.setEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

    @Override
    public double getPositionBlockingRad() {
        return getPositionRad().getAsDouble();
    }
}
