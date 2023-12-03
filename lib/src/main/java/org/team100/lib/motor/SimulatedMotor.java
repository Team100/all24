package org.team100.lib.motor;

public class SimulatedMotor<T> implements Motor100<T> {
    private double m_velocity = 0;

    @Override
    public double get() {
        // TODO: this is wrong
        return m_velocity;
    }

    @Override
    public void setDutyCycle(double output) {
        // ignore it
    }

    @Override
    public void setVelocity(double velocity, double accel) {
        m_velocity = velocity;
        // ignore accel
    }

    public double getVelocity() {
        return m_velocity;
    }
    
}
