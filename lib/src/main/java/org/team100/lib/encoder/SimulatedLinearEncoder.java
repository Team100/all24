package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

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
    private final SimulatedMotor<Distance100> m_motor;
    private final double m_reduction;
    private final double m_lowerLimit;
    private final double m_upperLimit;
    // accumulates.
    private double m_position = 0;
    private double m_time = Timer.getFPGATimestamp();

    /**
     * @param motor
     * @param reduction  ratio between motor and encoder
     * @param lowerLimit in m or rad
     * @param upperLimit in m or rad
     */
    public SimulatedLinearEncoder(
            Logger parent,
            SimulatedMotor<Distance100> motor,
            double reduction,
            double lowerLimit,
            double upperLimit) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_reduction = reduction;
        m_lowerLimit = lowerLimit;
        m_upperLimit = upperLimit;
        reset();
    }

    @Override
    public OptionalDouble getPosition() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double m_rate = m_motor.getVelocity() / m_reduction;
        m_position += m_rate * dt;
        m_position = MathUtil.clamp(m_position, m_lowerLimit, m_upperLimit);
        m_time = now;
        m_logger.logDouble(Level.TRACE, "position", () -> m_position);
        return OptionalDouble.of(m_position);
    }

    @Override
    public OptionalDouble getRate() {
        double m_rate = m_motor.getVelocity() / m_reduction;
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
