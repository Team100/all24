package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.telemetry.Telemetry.Level;

/** encoder implementation that supports only velocity measurement. */
public class VelocityBareEncoder implements IncrementalBareEncoder {
    private final SupplierLogger2 m_logger;
    private final BareMotor m_motor;
    // LOGGERS
    private final OptionalDoubleLogger m_log_position;
    private final OptionalDoubleLogger m_log_velocity;

    public VelocityBareEncoder(
            SupplierLogger2 parent,
            BareMotor motor) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_log_position = m_logger.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = m_logger.optionalDoubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    @Override
    public OptionalDouble getVelocityRad_S() {
        double m_rate = m_motor.getVelocityRad_S();
        return OptionalDouble.of(m_rate);
    }

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.empty();
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setEncoderPositionRad(double motorPositionRad) {
        m_motor.setEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log( this::getPositionRad);
        m_log_velocity.log( this::getVelocityRad_S);
    }

}
