package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.BareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

public class SimulatedBareEncoder implements IncrementalBareEncoder {
    private final SupplierLogger m_logger;
    private final BareMotor m_motor;

    // accumulates.
    private double m_position = 0;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedBareEncoder(
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
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double m_rate = m_motor.getVelocityRad_S();
        m_position += m_rate * dt;
        m_time = now;
        m_logger.logDouble(Level.TRACE, "position (m)", () -> m_position);
        return OptionalDouble.of(m_position);
    }

    @Override
    public void reset() {
        m_position = 0;
        m_time = Timer.getFPGATimestamp();
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
