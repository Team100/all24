package org.team100.lib.encoder;

import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated encoder that integrates motor velocity to get position.
 * 
 * Relies on Timer.getFPGATimestamp() to compute rate, which means you should
 * use SimHooks.stepTimingAsync() in your tests.
 */
public class SimulatedEncoder<T extends Measure100> implements Encoder100<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final SimulatedMotor<T> m_motor;
    private final double m_reduction;
    private final double m_lowerLimit;
    private final double m_upperLimit;
    // accumulates.
    private double m_position = 0;
    private double m_rate = 0;
    private double m_time = Timer.getFPGATimestamp();

    /**
     * @param name      may not start with a slash
     * @param motor
     * @param reduction ratio between motor and encoder
     * @param lowerLimit in m or rad
     * @param upperLimit in m or rad
     */
    public SimulatedEncoder(
            String name,
            SimulatedMotor<T> motor,
            double reduction,
            double lowerLimit,
            double upperLimit) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_motor = motor;
        m_reduction = reduction;
        m_lowerLimit = lowerLimit;
        m_upperLimit = upperLimit;
        reset();
    }

    @Override
    public Double getPosition() {
        return m_position;
    }

    @Override
    public double getRate() {
        return m_rate;
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
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        m_rate = m_motor.getVelocity() / m_reduction;
        m_position += m_rate * dt;
        m_position = MathUtil.clamp(m_position, m_lowerLimit, m_upperLimit);
        m_time = now;
        t.log(Level.TRACE, m_name, "position", m_position);
        t.log(Level.TRACE, m_name, "rate", m_rate);
    }
}
