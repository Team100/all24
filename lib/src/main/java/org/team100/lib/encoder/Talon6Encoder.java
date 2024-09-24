package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.telemetry.Telemetry.Level;

public class Talon6Encoder implements IncrementalBareEncoder {
    private final Talon6Motor m_motor;
    private final OptionalDoubleLogger m_log_position;
    private final OptionalDoubleLogger m_log_velocity;

    public Talon6Encoder(
            SupplierLogger2 parent,
            Talon6Motor motor) {
        SupplierLogger2 child = parent.child(this);
        m_motor = motor;
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.optionalDoubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    /** Nearly cached. */
    @Override
    public OptionalDouble getVelocityRad_S() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityRad_S = motorVelocityRev_S * 2 * Math.PI;
        return OptionalDouble.of(velocityRad_S);
    }

    /** Nearly cached. */
    @Override
    public OptionalDouble getPositionRad() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionRad = motorPositionRev * 2 * Math.PI;
        return OptionalDouble.of(positionRad);
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        m_motor.close();
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

}
