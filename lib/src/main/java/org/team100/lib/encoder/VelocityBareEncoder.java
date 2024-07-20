package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.BareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

/** encoder implementation that supports only velocity measurement. */
public class VelocityBareEncoder implements IncrementalBareEncoder {
    private final SupplierLogger m_logger;
    private final BareMotor m_motor;

    public VelocityBareEncoder(
            SupplierLogger parent,
            BareMotor motor) {
        m_logger = parent.child(this);
        m_motor = motor;
    }

    @Override
    public OptionalDouble getVelocityRad_S() {
        double m_rate = m_motor.getVelocityRad_S();
        m_logger.logDouble(Level.TRACE, "velocity (rad_s)", () -> m_rate);
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
        m_logger.logOptionalDouble(Level.TRACE, "position (rad)", this::getPositionRad);
        m_logger.logOptionalDouble(Level.TRACE, "velocity (rad_s)", this::getVelocityRad_S);
    }

}
