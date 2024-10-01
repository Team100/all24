package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motor.BareMotor;

/** encoder implementation that supports only velocity measurement. */
public class VelocityBareEncoder implements IncrementalBareEncoder {
    private final BareMotor m_motor;
    // LOGGERS
    private final OptionalDoubleLogger m_log_position;
    private final OptionalDoubleLogger m_log_velocity;

    public VelocityBareEncoder(
            LoggerFactory parent,
            BareMotor motor) {
        LoggerFactory child = parent.child(this);
        m_motor = motor;
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.optionalDoubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    /** Cached. */
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
        m_log_position.log(this::getPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

    @Override
    public double getPositionBlockingRad() {
        return getPositionRad().getAsDouble();
    }

}
