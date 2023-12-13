package org.team100.lib.encoder;

import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure;

import edu.wpi.first.wpilibj.Timer;

public class SimulatedEncoder<T extends Measure> implements Encoder100<T> {
    private final Telemetry t = Telemetry.get();
    private final T m_measure;
    private final String m_name;
    private final SimulatedMotor<T> m_motor;
    private final double m_reduction;
    private double m_position = 0;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedEncoder(T measure, String name, SimulatedMotor<T> motor, double reduction) {
        m_measure = measure;
        m_name = name + "/simulated_encoder";
        m_motor = motor;
        m_reduction = reduction;
    }

    @Override
    public double getPosition() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        m_position += getRate() * dt;
        m_time = now;
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
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getAbsolutePosition() {
        return m_measure.modulus(getPosition());
    }
}
