package org.team100.lib.encoder;

import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;

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
        m_name = String.format("/%s/Simulated Encoder", name);
        m_motor = motor;
        m_reduction = reduction;
        m_lowerLimit = lowerLimit;
        m_upperLimit = upperLimit;
    }

    @Override
    public double getPosition() {
        t.log(Level.DEBUG, m_name + "/position", m_position);
        return m_position;
    }

    @Override
    public double getRate() {
        double rate = m_motor.getVelocity() / m_reduction;
        t.log(Level.DEBUG, m_name + "/rate", rate);
        return rate;
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
        m_position += getRate() * dt;
        m_position = MathUtil.clamp(m_position, m_lowerLimit, m_upperLimit);
        m_time = now;
    }
}
