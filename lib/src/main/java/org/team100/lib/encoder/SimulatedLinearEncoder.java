package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated encoder that integrates motor velocity to get position.
 * 
 * Relies on Timer.getFPGATimestamp() to compute rate, which means you should
 * use SimHooks.stepTimingAsync() in your tests.
 */
public class SimulatedLinearEncoder implements IncrementalLinearEncoder {
    private final Logger m_logger;
    private final LinearMechanism m_motor;
    private final double m_lowerLimit;
    private final double m_upperLimit;
    // accumulates.
    private double m_position = 0;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedLinearEncoder(
            Logger parent,
            LinearMechanism motor,
            double lowerLimit,
            double upperLimit) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_lowerLimit = lowerLimit;
        m_upperLimit = upperLimit;
        reset();
    }

    @Override
    public OptionalDouble getPosition() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double m_rate = m_motor.getVelocityM_S();
        m_position += m_rate * dt;
        m_position = MathUtil.clamp(m_position, m_lowerLimit, m_upperLimit);
        m_time = now;
        m_logger.logDouble(Level.TRACE, "position", () -> m_position);
        return OptionalDouble.of(m_position);
    }

    @Override
    public OptionalDouble getRate() {
        double m_rate = m_motor.getVelocityM_S();
        m_logger.logDouble(Level.TRACE, "rate", () -> m_rate);
        return OptionalDouble.of(m_rate);
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
}
