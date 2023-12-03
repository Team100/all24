package org.team100.lib.encoder;

import org.team100.lib.motor.SimulatedMotor;

import edu.wpi.first.wpilibj.Timer;

public class SimulatedEncoder<T> implements Encoder100<T> {
    private final SimulatedMotor<T> m_motor;
    private double m_position = 0;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedEncoder(SimulatedMotor<T> motor) {
        m_motor = motor;
    }

    @Override
    public double getPosition() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        m_position += getRate() * dt;
        m_time = now;
        return m_position;
    }

    @Override
    public double getRate() {
        return m_motor.getVelocity();
    }

    @Override
    public void reset() {
        m_position = 0;
    }

    @Override
    public void close() {
    }
}
