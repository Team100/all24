package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

public class Talon6Encoder implements IncrementalBareEncoder {
    private final Logger m_logger;
    private final Talon6Motor m_motor;

    public Talon6Encoder(
            Logger parent,
            Talon6Motor motor) {
        m_logger = parent.child(this);
        m_motor = motor;
    }

    @Override
    public OptionalDouble getVelocityRad_S() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityRad_S = motorVelocityRev_S * 2 * Math.PI;
        m_logger.logDouble(Level.TRACE, "velocity (rev_s)", () -> motorVelocityRev_S);
        m_logger.logDouble(Level.TRACE, "velocity (rad_s)", () -> velocityRad_S);
        return OptionalDouble.of(velocityRad_S);
    }

    @Override
    public OptionalDouble getPositionRad() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionRad = motorPositionRev * 2 * Math.PI;
        m_logger.logDouble(Level.TRACE, "position (rev)", () -> motorPositionRev);
        m_logger.logDouble(Level.TRACE, "position (rad)", () -> positionRad);
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

}
