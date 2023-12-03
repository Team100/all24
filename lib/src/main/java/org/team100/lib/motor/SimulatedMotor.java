package org.team100.lib.motor;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

public class SimulatedMotor<T> implements Motor100<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;

    public SimulatedMotor(String name) {
        m_name = name + "/simulated_motor";
    }

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
        t.log(Level.DEBUG, m_name + "/velocity", velocity);

        m_velocity = velocity;
        // ignore accel
    }

    public double getVelocity() {
        return m_velocity;
    }

}
